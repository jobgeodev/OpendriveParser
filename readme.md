# Convert OpenDrive To Json 

1. ##  xsd to python class

https://pypi.org/project/generateDS/

```python
generateDS.py -o opendrive_parser_14H.py OpenDRIVE_1.4H.xsd

generateDS.py -o opendrive_parser_15M.py OpenDRIVE_1.5M.xsd
```

2. ## Convert

```python
xodr_file = r'sample_largezone.xodr'
opendrive = opendrive_parser.parse(xodr_file, silence=True)

convert = ConvertOpenDrive(opendrive, 0.5)

convert.json_all_baseroad(r'./tmp/all_baseroad_new.json')
```

