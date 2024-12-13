from config import api_key
import requests



class GeminiAdapter: 
    __gemini_url = 'https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent?key=' + api_key
# TODO store history of messages with role player / bot
    def __init__(self, initMessage):

        res = requests.post(self.__gemini_url, json = self.toPythonRequest(initMessage))

        if res.status_code != 200:
            raise Exception("Init Request to Gemini failed: "+ res.text)
    
    def toPythonRequest(self, text):
        return {
            "contents": [{
                "parts": [{"text": text}]
            }]
        }
    
    def request(self, text):
        res = requests.post(self.__gemini_url, json = self.toPythonRequest(text))

        if res.status_code != 200:
            raise Exception("Request to Gemini failed: "+ res.text)

        return res.json()['candidates'][0]['content']['parts'][0]['text']

if __name__ == '__main__':
    GeminiAdapter()