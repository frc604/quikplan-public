# *Quikplan*
![quikplan](https://github.com/frc604/quikplan-public/blob/main/resources/quikplan.png)

<p align="center">
<img src="https://github.com/frc604/quikplan-public/blob/main/resources/barrel_racing.gif" height="500"/>
</p>

## Quikstart

### Requirements
* [Python](https://www.python.org/downloads/) 3.7 or 3.8

Note: Quikplan has been **tested** with Python version `3.7` and `3.8`. Older versions **may** work but Python `3.9` is not currently supported.

### Installation
* Clone the repository to your local machine with `git clone https://github.com/frc604/quikplan-public.git`
* Navigate into the cloned repository with `cd quikplan-public`
* Install the necessary libraries through `pip` by running `pip3 install --upgrade -r requirements.txt` in the root folder

### Running
There are currently two available optimizers: `slalom` and `barrel_racing`. To run them just run their corresponding python file:
 * `python quikplan.py --slalom`
 * `python quikplan.py --barrel_racing`

## Initialization
Quikplan also includes a simple GUI for creating initialization paths to be optimized. This can be launched by running:
* `python quikplan.py --init`

<p align="center">
<img src="https://github.com/frc604/quikplan-public/blob/main/resources/init_gui.png" height="300"/>
</p>

Once the GUI has launched you can click on any of the preloaded setups in the obstacles list to load it onto the chart.

To add a waypoint just left click anywhere on the embedded chart and the line will expand to include the point you clicked.

Clicking and dragging lets you move a previously placed waypoint.

Right clicking on a previously placed waypoint deletes it from the path.

After creating an initialization path you can save it to a json file by clicking the `Save to JSON` button and entering the json name.

## Visualization
Quikplan offers numerous options to visualize optimized paths. The two main methods for this are `plot_traj` and `anim_traj` in `helpers.py`.

### `plot_traj` 
`plot_traj` will plot the x, y, and theta poses in a trajectory using the provided robot geometry. It can also be used to plot obstacles. See `quikplan_slalom.py` for example use cases. 

If the `save_png` param of `plot_traj` is set to `True` Quikplan will save the plot to a png file in the `renders` folder.

### `anim_traj` 
`anim_traj` will animate the robot moving through the poses in a trajectory using the provided robot geometry. It also has support for plotting obstacles. `quikplan_slalom.py` has an example use case.

Note: The timestep in `anim_traj` should be set to the dt between the trajectory states (in milliseconds) for the animation to run at the rate of the trajectory. `interp_state_vector` can be used to convert an optimized trajectory to a desired dt.

If the `save_gif` param of `anim_traj` is set to `True` Quikplan will save the animated trajectory as a gif file in the `renders` folder.
