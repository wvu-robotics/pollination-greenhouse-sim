# WVU Robotics Greenhouse Model

This repository wraps the greenhouse Gazebo model as a package. Sourcing from
a workspace including the package will add the `models` directory to
`GAZEBO_MODEL_PATH`, allowing the greenhouse model to be included like any other
Gazebo model.

# view world

roslaunch simulator-greenhouse greenhouse.launch

# generate more trees
Run python3 script from within models folder. Numpy and Pathlib dependencies may need to be installed

```
python3 tree_generator.py 20 
#to generate 20 trees, change number to generate differing amounts
```

Edit the sdf in greenhouse-textured to change the number of generated trees in the simulation. 
If you want to change flower distribution patterns, you can edit some parameters in the python file. 
