import jsonpickle
from json import JSONEncoder
import json


with open('data.json') as json_pickle:
    data = json.load(json_pickle)
    data_decoded = jsonpickle.decode(data)

    print(data_decoded)
