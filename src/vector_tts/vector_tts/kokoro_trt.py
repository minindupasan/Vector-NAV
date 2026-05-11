"""
Kokoro TTS — TensorRT Inference Engine
=======================================
Replaces KPipeline's PyTorch neural network inference with TensorRT engines
for GPU-accelerated synthesis on Jetson Orin.

Engines (FP16, built with trtexec):
  bert.engine              — BERT + bert_encoder
  text_encoder.engine      — Text encoder
  predictor_duration.engine — Duration predictor (text_encoder + LSTM + duration_proj)
  predictor_f0n.engine     — F0 and noise predictor
  decoder.engine           — iSTFTNet decoder → waveform

The G2P (grapheme-to-phoneme) pipeline is still handled by misaki/kokoro.
"""

import json
import logging
import numpy as np
import torch
import tensorrt as trt
from pathlib import Path
from huggingface_hub import hf_hub_download

logger = logging.getLogger(__name__)

# TensorRT dtype → numpy dtype
_TRT_TO_NP = {
    trt.DataType.FLOAT: np.float32,
    trt.DataType.HALF: np.float16,
    trt.DataType.INT32: np.int32,
    trt.DataType.INT64: np.int64,
    trt.DataType.BOOL: np.bool_,
}


class _TRTRunner:
    """Thin wrapper around a single TensorRT engine."""

    def __init__(self, engine_path: str):
        self._logger = trt.Logger(trt.Logger.WARNING)
        runtime = trt.Runtime(self._logger)
        with open(engine_path, 'rb') as f:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError(f"Failed to create TensorRT execution context for {engine_path}. This usually means the system is Out of Memory (OOM).")
        self.stream = torch.cuda.Stream()

        # Catalog I/O tensors
        self.inputs = {}
        self.outputs = {}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            dtype = _TRT_TO_NP[self.engine.get_tensor_dtype(name)]
            if mode == trt.TensorIOMode.INPUT:
                self.inputs[name] = dtype
            else:
                self.outputs[name] = dtype

    def __call__(self, **kwargs) -> dict:
        """
        Run inference. Pass input tensors as keyword arguments (numpy arrays or torch tensors).
        Returns dict of output name → numpy array.
        """
        # Set input shapes and bind input buffers
        gpu_buffers = {}
        for name, dtype in self.inputs.items():
            arr = kwargs[name]
            if isinstance(arr, torch.Tensor):
                t = arr.cuda()
            else:
                if arr.dtype != dtype:
                    arr = arr.astype(dtype)
                t = torch.from_numpy(arr).cuda()
            
            self.context.set_input_shape(name, t.shape)
            gpu_buffers[name] = t
            self.context.set_tensor_address(name, t.data_ptr())

        # Allocate output buffers
        results = {}
        for name, dtype in self.outputs.items():
            shape = self.context.get_tensor_shape(name)
            t = torch.empty(tuple(shape), dtype=torch.float32, device='cuda')
            gpu_buffers[name] = t
            self.context.set_tensor_address(name, t.data_ptr())

        # Execute
        self.context.execute_async_v3(self.stream.cuda_stream)
        self.stream.synchronize()

        # Copy outputs to CPU
        for name in self.outputs:
            results[name] = gpu_buffers[name].cpu().numpy()

        return results


class KokoroTRT:
    """
    TensorRT-accelerated Kokoro TTS inference.

    Usage:
        tts = KokoroTRT('/home/admin/models/kokoro_trt')
        audio = tts.synthesize("Hello world", voice='af_heart', speed=1.0)
    """

    def __init__(self, engine_dir: str, repo_id: str = 'hexgrad/Kokoro-82M'):
        engine_dir = Path(engine_dir)

        # Load vocab from config
        config_path = hf_hub_download(repo_id=repo_id, filename='config.json')
        with open(config_path) as f:
            config = json.load(f)
        self.vocab = config['vocab']

        # Load TRT engines (fall back to PyTorch decoder if FP32 engine missing)
        logger.info('Loading Kokoro TRT engines...')
        self._bert = _TRTRunner(str(engine_dir / 'bert.engine'))
        self._text_enc = _TRTRunner(str(engine_dir / 'text_encoder.engine'))
        self._pred_dur = _TRTRunner(str(engine_dir / 'predictor_duration.engine'))
        self._pred_f0n = _TRTRunner(str(engine_dir / 'predictor_f0n.engine'))

        decoder_fp32 = engine_dir / 'decoder_fp32.engine'
        decoder_fp16 = engine_dir / 'decoder.engine'
        if decoder_fp32.exists():
            self._decoder = _TRTRunner(str(decoder_fp32))
            logger.info('Using FP32 decoder engine')
        elif decoder_fp16.exists():
            self._decoder = _TRTRunner(str(decoder_fp16))
            logger.info('Using FP16 decoder engine (may have NaN issues)')
        else:
            self._decoder = None
            logger.error('No decoder engine found — TTS synthesis will fail')

        self._repo_id = repo_id

        logger.info('Kokoro TRT engines loaded')

        # Load G2P for English
        from misaki import en, espeak
        try:
            fallback = espeak.EspeakFallback(british=False)
        except Exception:
            fallback = None
        self.g2p = en.G2P(trf=False, british=False, fallback=fallback, unk='')

        # Voice cache
        self._voices = {}
        self.sample_rate = 24000

    def load_voice(self, voice: str) -> np.ndarray:
        """Load and cache a voice pack. Returns numpy array [N, 256]."""
        if voice in self._voices:
            return self._voices[voice]
        f = hf_hub_download(repo_id=self._repo_id, filename=f'voices/{voice}.pt')
        pack = torch.load(f, weights_only=True).numpy()
        self._voices[voice] = pack
        return pack

    def phonemize(self, text: str) -> list[tuple[str, str]]:
        """Convert text to list of (graphemes, phonemes) chunks."""
        _, tokens = self.g2p(text)
        chunks = []
        for gs, ps, _ in self._en_tokenize(tokens):
            if ps:
                if len(ps) > 510:
                    ps = ps[:510]
                chunks.append((gs, ps))
        return chunks

    def synthesize(
        self, text: str, voice: str = 'af_heart', speed: float = 1.0
    ) -> np.ndarray:
        """
        Full text-to-speech: text → phonemes → TRT inference → audio.
        Returns float32 audio array at 24kHz.
        """
        pack = self.load_voice(voice)
        chunks = self.phonemize(text)
        audio_parts = []
        for _, ps in chunks:
            audio = self._infer(ps, pack, speed)
            if audio is not None:
                audio_parts.append(audio)
        if not audio_parts:
            return np.zeros(0, dtype=np.float32)
        return np.concatenate(audio_parts)

    def synthesize_phonemes(
        self, ps: str, voice: str = 'af_heart', speed: float = 1.0
    ) -> np.ndarray | None:
        """Synthesize from pre-computed phonemes (single chunk)."""
        pack = self.load_voice(voice)
        return self._infer(ps, pack, speed)

    def _infer(self, ps: str, pack: np.ndarray, speed: float) -> np.ndarray | None:
        """Run TRT inference for a single phoneme chunk."""
        # 1. Phonemes → input_ids with BOS/EOS
        ids = [self.vocab[p] for p in ps if p in self.vocab]
        if not ids:
            return None
        ids = [0] + ids + [0]
        seq_len = len(ids)

        # 2. Prepare common inputs
        input_ids = np.array([ids], dtype=np.int64)
        input_lengths = np.array([seq_len], dtype=np.int64)
        text_mask = np.zeros([1, seq_len], dtype=np.bool_)
        attention_mask = np.ones([1, seq_len], dtype=np.int32)

        # 3. Voice reference — indexed by phoneme length
        ref_s = pack[min(len(ps) - 1, len(pack) - 1)]  # [1, 256] or [256]
        ref_s = ref_s.reshape(256)
        s = ref_s[128:].reshape(1, 128).astype(np.float32)   # prosody style
        style = ref_s[:128].reshape(1, 128).astype(np.float32)  # decoder style

        # 4. BERT → d_en [1, 512, seq_len]
        bert_out = self._bert(input_ids=input_ids, attention_mask=attention_mask)
        d_en = bert_out['d_en']

        # 5. Text encoder → t_en [1, 512, seq_len]
        te_out = self._text_enc(
            input_ids=input_ids, input_lengths=input_lengths, text_mask=text_mask
        )
        t_en = te_out['t_en']

        # 6. Predictor duration → d [1, seq_len, 640], duration [1, seq_len]
        pd_out = self._pred_dur(
            d_en=d_en, s=s, input_lengths=input_lengths, text_mask=text_mask
        )
        d = pd_out['d']            # [1, seq_len, 640]
        duration = pd_out['duration']  # [1, seq_len] — already sigmoid'd and summed

        # 7. Process duration → alignment matrix
        pred_dur = np.round(duration / speed).clip(1).astype(np.int64).squeeze()  # [seq_len]
        if pred_dur.ndim == 0:
            pred_dur = pred_dur.reshape(1)
        indices = np.repeat(np.arange(seq_len), pred_dur)
        total_len = len(indices)
        if total_len == 0:
            return None

        pred_aln = np.zeros((seq_len, total_len), dtype=np.float32)
        pred_aln[indices, np.arange(total_len)] = 1.0
        pred_aln = pred_aln[np.newaxis]  # [1, seq_len, total_len]

        # 8. en = d.transpose @ pred_aln → [1, 640, total_len]
        # d is [1, seq_len, 640], d transposed is [1, 640, seq_len]
        d_t = np.transpose(d, (0, 2, 1))  # [1, 640, seq_len]
        en = d_t @ pred_aln  # [1, 640, total_len]

        # 9. F0 and noise prediction
        f0n_out = self._pred_f0n(en=en.astype(np.float32), s=s)
        f0 = f0n_out['F0']   # [1, 2*total_len]
        noise = f0n_out['N']  # [1, 2*total_len]

        # 10. ASR features via alignment
        # asr = t_en @ pred_aln → [1, 512, total_len]
        asr = t_en @ pred_aln  # [1, 512, total_len]

        # 11. Decode → audio (TRT).
        if self._decoder is None:
            logger.error('No TensorRT decoder engine available')
            return np.zeros(0, dtype=np.float32)

        # Generate audio-rate randoms for the source module (replaces the
        # torch.randn_like calls that TRT 10 cannot compile).
        audio_len = f0.shape[1] * 300  # f0_upsamp scale_factor
        randn_sine_t = torch.randn((1, audio_len, 9), dtype=torch.float32, device='cuda')
        
        dec_out = self._decoder(
            asr=asr.astype(np.float32),
            f0=f0.astype(np.float32),
            noise=noise.astype(np.float32),
            style=style,
            randn_sine=randn_sine_t,
        )
        audio = dec_out['audio'].squeeze().astype(np.float32)
        return np.nan_to_num(audio, nan=0.0, posinf=0.0, neginf=0.0)

    # ── G2P chunking (from KPipeline) ────────────────────────────────────

    @staticmethod
    def _tokens_to_ps(tokens) -> str:
        return ''.join(
            t.phonemes + (' ' if t.whitespace else '') for t in tokens
        ).strip()

    @staticmethod
    def _tokens_to_text(tokens) -> str:
        return ''.join(t.text + t.whitespace for t in tokens).strip()

    def _en_tokenize(self, tokens):
        tks = []
        pcount = 0
        for t in tokens:
            t.phonemes = '' if t.phonemes is None else t.phonemes
            next_ps = t.phonemes + (' ' if t.whitespace else '')
            next_pcount = pcount + len(next_ps.rstrip())
            if next_pcount > 510:
                z = self._waterfall_last(tks, next_pcount)
                text = self._tokens_to_text(tks[:z])
                ps = self._tokens_to_ps(tks[:z])
                yield text, ps, tks[:z]
                tks = tks[z:]
                pcount = len(self._tokens_to_ps(tks))
                if not tks:
                    next_ps = next_ps.lstrip()
            tks.append(t)
            pcount += len(next_ps)
        if tks:
            text = self._tokens_to_text(tks)
            ps = self._tokens_to_ps(tks)
            yield text, ps, tks

    @staticmethod
    def _waterfall_last(tokens, next_count):
        waterfall = ['!.?…', ':;', ',—']
        bumps = [')', '"']
        for w in waterfall:
            z = next(
                (i for i, t in reversed(list(enumerate(tokens)))
                 if t.phonemes in set(w)), None
            )
            if z is None:
                continue
            z += 1
            if z < len(tokens) and tokens[z].phonemes in bumps:
                z += 1
            if next_count - len(KokoroTRT._tokens_to_ps(tokens[:z])) <= 510:
                return z
        return len(tokens)
