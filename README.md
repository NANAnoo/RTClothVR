# RTClothVR

This project is used for learning real-time cloth simulation and exploring the feasiablity of performing real-time cloth simulation for VR applications. The cloth model is basically comes from:

>"
D. Baraff and A. Witkin, “Large steps in cloth simulation,” Proceedings of the 25th annual conference on Computer graphics and interactive techniques  - SIGGRAPH ’98, 1998, doi: https://doi.org/10.1145/280814.280821."

## Integrators and Their Support Hardware

| Integrator | CPU | GPU Computer Shader|
|------------|-----|--------------------|
| Implicit | :white_check_mark: | :white_large_square: |
| LeapFrog | :white_check_mark:  | :white_large_square: |
| Verlet | :white_check_mark:  | :white_check_mark: |

## Support Features

- [x] Cloth Self Collision Detection and Responce
- [x] Cloth-Object Collision Detection and Responce
- [x] Positional Constraints
  - [x] Fix on plane
  - [x] Fix on line
  - [x] Fix position

- [x] Velocity Constraints
- [x] External forces

## How to use this plugin

- Download UE4.

- `git clone https://github.com/NANAnoo/RTClothVR.git`
- Launch the UE4 Editor by double click `RTClothVR.uproject`.
- Add Component `RTClothMesh` to one of a static mesh actor in the game scene. It should look like the figure below:
<img src='./screenshoots/LoadTShirt.png'>
- Simulation parameters can be modified in `Cloth Parameters` section of the detail panel.

## Some of the results


Positional Constraints
![Alt text](./screenshoots/image.png)

Velocity Constraints
![Alt text](./screenshoots/image-1.png)

External Collision
![Alt text](./screenshoots/Collision1.png)

Internal Collision
![Alt text](./screenshoots/Collision2.png)

Simulating three pieces of cloth which size is 64 x 64
![Alt text](./screenshoots/3x64x64.png)