from dotenv import load_dotenv
from openai import OpenAI

load_dotenv()

openai = OpenAI()


def get_command(statement):
    completion = openai.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {
                "role": "system",
                "content": "Classify the user's statement as one of the following commands: '<sleep>' (anything related to going to bed or sleeping), '<trick>' (anything related to a robot dog cute trick), or '<unknown>' (if it doesn't match a command).",
            },
            {"role": "user", "content": statement},
        ],
    )

    return completion.choices[0].message.content
