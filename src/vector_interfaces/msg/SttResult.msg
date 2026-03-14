std_msgs/Header header
string text           # transcribed speech
float32 confidence    # recognition confidence 0.0–1.0, -1.0 if unavailable
string language       # detected language code e.g. 'en', empty if unknown
