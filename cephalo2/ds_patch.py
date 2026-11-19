import json,sys
s=json.load(open(sys.argv[1]))["content"] if sys.argv[1].endswith(".json") else open(sys.argv[1]).read()
