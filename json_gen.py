'''

Author: Jojo Meunier jmeunier@bu.edu 4/10/16

File to generate the json doc that contains 
user defined workspace configuration 

The workspace configuration allows the user to define where the bench 
bot will have to move to complete a specified task

*****All dicts that contain dimensions are in units of mm for size dimensions 
	and degrees for coordinate positions in the x,y,z plane*******

It currently generates a json file with the following definitions:

Tip box object{
    Footprint [x,y]
    Dimensions [length, width, height] //for now, just stick to defining cubes/rectangles
    Location [x,y,z] //most likely a coordinate of one of the corners of the cube
    Number of tips = 96 //alternatively, this can be calculated from the tip grid dimensions. might as well include it though.
    Tip Grid Dimensions [8 rows, 12 columns]//the tips in a tip box are arranged in a grid
    Tip spacing [row spacing, column spacing]
    Relative location of the first tip [x,y,z] //location of the first tip relative to one of the corners of the tip box
}

Waste Container{
    Footprint [x,y]
    Dimensions [length, width, height]
    Location [x,y,z]
    Container hole dimensions [x,y]
    Relative location of hole edge[x,y,]
}

Tube rack{
    Footprint [x,y]
    Dimensions [length, width, height]
    Location [x,y,z]
    Number of wells = 90
    Well Grid Dimensions [5 rows, 18 columns]
    Well Spacing [row spacing, column spacing]
    Relative location of the first well [x,y]
    Height of Tubes being used with the tube rack  = 20 //all dimension units in mm unless otherwise stated
}

Microplate{
    Footprint [x,y]
    Dimensions [length, width, height]
    Location [x,y,z]
    Number of wells = 96
    Well Grid Dimensions [8 rows, 12 columns]
    Well Spacing [row spacing, column spacing]
    Relative location of the first well [x,y]
}

'''

import json 
import sys
import os 

class JSON:

	'''
	Import NOTE for modifying the dimensions: length corresponds to the z coordinate in a 3d plane
	while height correponds to the y coordinte in the 3d plane. The width corresponds to x

	'''

	filepath = sys.argv[1]
	data_array = []

	def __init__(self):
		print ("\nInitializing new json object..")

	def loadfile(self):
		with open(JSON.filepath) as data_file:
			self.data = json.load(data_file)
		print ("loaded json file to edit...")

	def write_tip_box(self):

		Footprint = {"x":1,"y":2}
		Dimensions = {"length":30,"width":12,"height":45}
		Location = {"x":3.0,"y":0.0,"z":-10.0}
		tipNum = {"tip_number":96}
		gridDimensions = {"rows":8,"columns":12}
		tipSpacing = {"row_spacing":2, "column_spacing":2}
		firstTipLocation = {"x":1,"y":1,"z":1}

		tipBoxObject = [{"Footprint": Footprint,"Dimensions":Dimensions,
						"Location":Location,"tipNum":tipNum,"gridDimensions":gridDimensions,
						"tipSpacing":tipSpacing,"firstTipLocation":firstTipLocation}]

		self.data = {"collection": "TipBox","parameters":tipBoxObject}
		JSON.data_array.append(self.data)


	def write_tube_rack(self):

		Footprint = {"x":1,"y":2}
		Dimensions = {"length":15,"width":6,"height":30}
		Location = {"x":-3.0,"y":0.0,"z":-5.0}
		wellNum = {"well_number":90}
		gridDimensions = {"rows":5,"columns":18}
		wellSpacing = {"row_spacing":2, "column_spacing":2}
		firstWellLocation = {"x":1,"y":1,"z":1}
		tubeHeight = {"height":20}

		tubeRackObject = [{"Footprint": Footprint,"Dimensions":Dimensions,
						"Location":Location,"tipNum":wellNum,"gridDimensions":gridDimensions,
						"tipSpacing":wellSpacing,"firstTipLocation":firstWellLocation,"tubeHeight":tubeHeight}]

		self.data = {"collection":"TubeRack", "parameters":tubeRackObject}
		JSON.data_array.append(self.data)



	def write_waste_container(self):
		
		Footprint = {"x":1,"y":2}
		Dimensions = {"length":5,"width":12,"height":12}
		Location = {"x":-5.0,"y":0.0,"z":-5.0}
		containerHole = {"x:":1, "y":1}
		holeEdgePosition = {"x":1,"y":1,"z":1}

		wasteContainerObject = [{"Footprint": Footprint,"Dimensions":Dimensions,
						"Location":Location,"containerHole":containerHole,"holeEdgePosition":holeEdgePosition}]

		self.data = {"collection":"WasteContainer", "parameters":wasteContainerObject}
		JSON.data_array.append(self.data)


	def write_micro_plate(self):
		
		Footprint = {"x":1,"y":2}
		Dimensions = {"length":10,"width":10,"height":10}
		Location = {"x":-2.0,"y":0.0,"z":-3.0}
		wellNum = {"tip_number":96}
		gridDimensions = {"rows":8,"columns":12}
		wellSpacing = {"row_spacing":2, "column_spacing":2}
		firstWellLocation = {"x":1,"y":1,"z":1}

		microPlateObject = [{"Footprint": Footprint,"Dimensions":Dimensions,
						"Location":Location,"tipNum":wellNum,"gridDimensions":gridDimensions,
						"tipSpacing":wellSpacing,"firstTipLocation":firstWellLocation}]

		self.data = {"collection":"MicroPlate","parameters":microPlateObject}
		JSON.data_array.append(self.data)


	def store_data(self):
		with open(JSON.filepath, 'w') as outfile:
			json.dump(JSON.data_array,outfile,sort_keys = True, indent =2)



new_json = JSON()
new_json.loadfile()
new_json.write_tip_box()
new_json.write_tube_rack()
new_json.write_micro_plate()
new_json.write_waste_container()
new_json.store_data()
