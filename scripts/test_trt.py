import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import numpy as np

class BGE_TRTRunner:
    def __init__(self, engine_path: str, max_batch=1, max_seq_len=256):
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(self.logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()
        
        self.inputs = []
        self.outputs = []
        
        # Setup buffers for maximum requested size
        self.context.set_input_shape("input_ids", (max_batch, max_seq_len))
        self.context.set_input_shape("attention_mask", (max_batch, max_seq_len))
        
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = self.context.get_tensor_shape(name)
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            size = trt.volume(shape) * np.dtype(dtype).itemsize
            
            host_mem = cuda.pagelocked_empty(trt.volume(shape), dtype)
            device_mem = cuda.mem_alloc(size)
            self.context.set_tensor_address(name, int(device_mem))
            
            tensor_info = {
                "name": name,
                "host": host_mem,
                "device": device_mem,
                "shape": shape,
                "dtype": dtype
            }
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.inputs.append(tensor_info)
            else:
                self.outputs.append(tensor_info)

    def run(self, input_ids_np, attention_mask_np):
        shape = input_ids_np.shape
        # We must set input shape exactly to what we're feeding
        self.context.set_input_shape("input_ids", shape)
        self.context.set_input_shape("attention_mask", shape)
        
        for inp in self.inputs:
            if inp["name"] == "input_ids":
                np.copyto(inp["host"][:input_ids_np.size], input_ids_np.ravel().astype(inp["dtype"]))
            elif inp["name"] == "attention_mask":
                np.copyto(inp["host"][:attention_mask_np.size], attention_mask_np.ravel().astype(inp["dtype"]))
            cuda.memcpy_htod_async(inp["device"], inp["host"], self.stream)
            
        self.context.execute_async_v3(self.stream.handle)
        
        for out in self.outputs:
            out_shape = self.context.get_tensor_shape(out["name"])
            out["current_shape"] = out_shape
            cuda.memcpy_dtoh_async(out["host"], out["device"], self.stream)
            
        self.stream.synchronize()
        
        out_dict = {}
        for out in self.outputs:
            # Reshape based on the actual shape computed by the context
            out_dict[out["name"]] = out["host"][:trt.volume(out["current_shape"])].reshape(out["current_shape"]).copy()
            
        return out_dict

if __name__ == "__main__":
    from transformers import AutoTokenizer
    tokenizer = AutoTokenizer.from_pretrained("/home/admin/vector_nav/src/vector_rag/vector_rag/data/onnx")
    runner = BGE_TRTRunner("/home/admin/vector_nav/src/vector_rag/vector_rag/data/onnx/model.engine")
    encoded = tokenizer(["Testing TensorRT integration for RAG"], padding='max_length', max_length=256, truncation=True, return_tensors="np")
    res = runner.run(encoded["input_ids"], encoded["attention_mask"])
    print(res["last_hidden_state"].shape)
