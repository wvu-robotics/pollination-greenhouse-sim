#Tree generator for Robot-Based Pollination, Written by Ibrahim Musaddequr Rahman at the WVU IRL

from xml.etree.ElementTree import Element, SubElement, Comment, tostring
from xml.dom import minidom
from functools import reduce
from numpy import random
from math import atan2
import sys
from pathlib import Path
# from https://pymotw.com/2/xml/etree/ElementTree/create.html
def prettify(elem):
	"""Return a pretty-printed XML string for the Element.
	"""
	rough_string = tostring(elem, encoding='utf-8',method='xml')
	reparsed = minidom.parseString(rough_string)
	return reparsed.toprettyxml(indent="  ")



def addflower(parent,n,flower_pose=[]):
	include = SubElement(parent,'include')
	name = SubElement(include,"name")
	name.text = 'bramble_'+str(n)
	uri = SubElement(include,"uri")
	uri.text = 'model://bramble'
	pose = SubElement(include,'pose')
	pose.text = reduce(lambda a,b: str(a)+" "+str(b),flower_pose,"")
	
	joint = SubElement(parent,'joint')
	joint.set('name','flower_joint_'+str(n))
	joint.set('type','fixed')
	child = SubElement(joint,'child')
	child.text = name.text+'::flower_link'
	parent = SubElement(joint,'parent')
	parent.text = 'link'

def generateTreeWithFlowers(skew_name,flower_pose_list=[[]]):
	sdf = Element('sdf')
	sdf.set('version','1.6')

	model = SubElement(sdf,'model')
	model.set('name','tree-flowers')
	model.append(Comment('Skew-'+skew_name))

	static = SubElement(model,'static')
	static.text = 'true';

	link = SubElement(model,'link')
	link.set('name','link')

	#tree-mesh
	include = SubElement(model,'include')
	uri = SubElement(include,"uri")
	uri.text = 'model://tree-mesh'
	pose = SubElement(include,'pose')
	pose.text = '0 0 0 0 0 0'

	#tree-mesh joint
	joint = SubElement(model,'joint')
	joint.set('name','tree_joint')
	joint.set('type','fixed')
	child = SubElement(joint,'child')
	child.text = 'tree-mesh::link'
	parent = SubElement(joint,'parent')
	parent.text = 'link'

	#I'm lazy, and for each is easier than writing out full iterator
	i = 0
	for flower_pose in flower_pose_list:
		#weird null getting added in at some point, checking instead of debugging
		if(len(flower_pose)>0):
			addflower(model,i,flower_pose)
			i+=1

	return sdf

#generate a random number within a mathematical interval. All bounds inclusive, except for highest(side effect of random.uniform)
#If multiple intervals passed, returns a number in the union of them(in order of low to high)
#uniform distribution for now, fancier ones could eventually be implemented
def randomInIntervals(intervals = [[]]):
	size = reduce(lambda a,b:a + (b[1]-b[0]),intervals,0) # find the size of intervals together
	rand = random.uniform(0,size) # generate on that generalized interval
	for interval in intervals: #rejustify to specific interval
		interval_size = interval[1] - interval[0]
		if rand > interval_size:
			rand -= interval_size
		else : return (interval[0] + rand)
	#for loop should never exit, if so some math or input was wrong

# from here https://stackoverflow.com/questions/952914/how-to-make-a-flat-list-out-of-a-list-of-lists
def flatten(t):
    return [item for sublist in t for item in sublist]
#used for testing to visualize x y z bounds
def generateBoundaryFlowerPoses(offset_bounds = [[[]]]):
	poses = [[]]
	for x in flatten(offset_bounds[0]):
		for y in flatten(offset_bounds[1]):
			for z in flatten(offset_bounds[2]):
				poses.append([x,y,z,0,0,0])
	return poses

def generateRandomFlowerPose(offset_bounds = [[[]]]):
	#pose is in the following order x y z r p y, in m and rad respectively
	x = randomInIntervals(offset_bounds[0])
	y = randomInIntervals(offset_bounds[1])
	z = randomInIntervals(offset_bounds[2])
	#flower is natively pointing upwards,
	#initial offsets are orient it normal to center of the tree
	PI = 3.14159265358979
	r = atan2(x,y) -PI/2 + randomInIntervals(offset_bounds[3])
	p = PI/2 + randomInIntervals(offset_bounds[4])
	yaw = randomInIntervals(offset_bounds[5]) #not that important, just petal positioning
	return [x,y,z,r,p,yaw]

def generateRandomTreeWithFlowers(flower_bounds=[[]],offset_bounds=[[[]]]):
	name = "Seed:" + str(random.get_state()[1][0])
	num_flowers = int(randomInIntervals(flower_bounds)) # could use randint, but this should also work
	flower_poses = [[]]
	for flower in range(0,num_flowers):
		flower_poses.append(generateRandomFlowerPose(offset_bounds))
	return generateTreeWithFlowers(name,flower_poses)

def generateModelConfig(model_name):
	model = Element('model')
	name = SubElement(model,'name')
	name.text = model_name
	version = SubElement(model,'version')
	version.text = '1.0'
	sdf = SubElement(model,'sdf')
	sdf.set('version','1.6')
	sdf.text=model_name+'.sdf'
	author = SubElement(model,'author')
	author_name = SubElement(author,'name')
	email = SubElement(author,'email')
	description = SubElement(model,'description')
	return model
#sdf generator test
# pose_list = [[0.1,0.1,0.2,0,0,0]]

# tree = generateTreeWithFlowers("new_skew",pose_list)
# print(prettify(tree))


# random interval tests
# print('simple')
# intervals_simple = [[0,4]]

# for x in range(0,10): print(randomInIntervals(intervals_simple))

# print('Complex')
# intervals_complex = [[-1,0.25],[2,6]]

# for x in range(0,10): print(randomInIntervals(intervals_complex))

#fancier shaped bounds might eventually be needed, but I think this style should cover most things
#bounds tested in sim
offset_bounds = [
  [[-0.15,-0.05],[0.05,0.12]]#x
, [[-0.08,-0.03],[0.05,0.13]]#y
, [[0.3,0.6]]#z
, [[-0.78,0.78]]#r
, [[-0.78,0.78]]#p
, [[0,0]]#yaw
]

boundary_tree = generateTreeWithFlowers("boundary",generateBoundaryFlowerPoses(offset_bounds))
#print(prettify(boundary_tree))

#nick wanted 4-7 per side
flower_bounds = [[8,14]]

#command line utility
if(len(sys.argv) <= 1) : print("Please specify the number of trees to generate as a command line argument")
else :
	num_trees = int(sys.argv[1])
	print('Generating '+str(num_trees)+' trees')
	
	for tree in range(0,num_trees):
		name = 'generated_tree_'+str(tree);
		random_tree = generateRandomTreeWithFlowers(flower_bounds,offset_bounds)
		random_tree_config = generateModelConfig(name)

		Path(name).mkdir(parents=True,exist_ok=True)
		model_file = open(name+'/'+name+'.sdf','w+')
		model_file.write(prettify(random_tree))
		model_file.close();

		config_file = open(name+'/model.config','w+')
		config_file.write(prettify(random_tree_config))
		config_file.close();
	name = 'generated_boundary_tree'
	boundary_tree_config = generateModelConfig(name)
	Path(name).mkdir(parents=True,exist_ok=True)
	model_file = open(name+'/'+name+'.sdf','w+')
	model_file.write(prettify(boundary_tree))
	model_file.close();

	config_file = open(name+'/model.config','w+')
	config_file.write(prettify(boundary_tree_config))
	config_file.close();


# random tree tester
# random_tree = generateRandomTreeWithFlowers(flower_bounds,offset_bounds)
#print(prettify(random_tree))

