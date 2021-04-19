# sc635-project
## Multi-UAV Surveillance With Minimum Information Idleness and Latency Constraints

## Dependencies:
python-tsp
networkx

## How to run code:
`python main_h1.py`

1. Check the Graph in image and change it in main_h1.py if required
2. Enter the Sensing location : _Example: 8_
3. Enter maximum Number of UAVs: _Example: 3_
4. Enter maximum Latency: _Example: 3_

### Note:
1. _Current implementation supports cooperative mission only one SL. This is due to limitation of available doccumentation of functions used by authors. We could not reproduce the functions as we couldn't find any clue how the stated functions in the paper work._ 
2. _We can easily extend this code to N SLs if we get the relevant function pseudocodes/logic._