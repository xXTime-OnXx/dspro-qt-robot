import google.generativeai as genai

class GeminiAdapter: 

    def __init__(self):
        genai.configure(api_key="AIzaSyCW_3vt1S1cr0pIUMqDU8W1-qRFTLo1jgg")
        self.model = genai.GenerativeModel("gemini-1.5-flash")
        self.generation_config = genai.types.GenerationConfig(
            candidate_count=1,
            max_output_tokens=20,
            temperature=1.0,
        ),
        #TODO: init Prompt
        response = self.model.generate_content("Explain how AI works")
        print(response)
    
if __name__ == '__main__':
    GeminiAdapter()