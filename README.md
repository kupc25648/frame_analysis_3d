# frame_analysis_3d
FEM analysis of frame structure and .obj export module

What the project does?

This is the Program to calculate 3D Frame Structure using Matrix Analysis Method. By giving input data of frame structures, loads and restrains, The program will form the structure stiffness matrix and will calculate deformations of the structure and strain energy. It also have export module which will export structure in .obj format. 

How users can get started with the project?

The repository includes 3 parts

FEM_frame.py
frame_GEN.py
frame_example.py
Users can create input data of frame structures, loads and restrains in "frame_example.py".

Where users can get help with your project?

Please feel free to contact me at kupc25648@hotmail.com

<script src="https://aframe.io/releases/0.7.0/aframe.min.js"></script>
<script src="https://jeromeetienne.github.io/AR.js/aframe/build/aframe-ar.js"></script>
<a-scene embedded arjs>
    <a-entity
      obj-model="obj: url(https://github.com/kupc25648/frame_analysis_3d/blob/master/Frame.obj)"
      rotation="0 0 0"
      scale="0.1 0.1 0.1"
    >
    </a-entity>
</a-scene>
