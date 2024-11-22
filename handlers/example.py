import eel
import random
import json

@eel.expose
def get_random_number():
    return random.randint(1, 100)

@eel.expose
def save_number(number):
    try:
        with open('data/numbers.json', 'r') as f:
            numbers = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        numbers = []
    
    numbers.append(number)
    
    with open('data/numbers.json', 'w') as f:
        json.dump(numbers, f)
    
    return numbers