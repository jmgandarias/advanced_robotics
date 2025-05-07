# Lab Session 3: Inverse Dynamics Control

In this lab session you will learn how to implement inverse dynamics controllers to compensate the non-linear dynamics of the manipulator to let you impose a specific  desired dynamic behavior of the manipulator. Note that you are expected to have finished the previous lab session as you'll need that implementation. 

## 3.1. Gravity compensation

### 3.1.1. Implementation
As you could have noticed in the previous lab, the gravitational effects on the manipulator makes it to "fall". Hence, out controller should include these effects when computing the joint torques that will be commanded to the actuators. The simplest inverse dynamics controller is the gravity compensation controller that set the torques of the actuators to be equal to those produced by gravity 

$$
\boldsymbol{\tau} = \mathbf{g}(\mathbf{q})
$$

To implement the gravity compensation controller you need to do the following:

1. Create the gravity compensation node: `gravity_compensation.cpp`

    <details>
    <summary>Show the code</summary>
    ```cpp title="gravity_compensation.cpp"
        --8<-- "snippets/lab3/gravity_compensation.cpp"
    ```
    </details>

2. You need to program the method `gravity_compensation()` to calculate de desired torques.
    ```cpp
    // Method to calculate the desired joint torques
    Eigen::VectorXd gravity_compensation()
    {
        // Placeholder for calculate the commanded torques
        // Calculate the control torque to compensate only for gravity effects: tau = g(q)

        // Calculate g_vect

        // // Calculate desired torque
        Eigen::VectorXd torque(2);
        torque << 0, 0;

        return torque;
    }
    ```
3. Create the `gravity_compensation_launch.py` file (you need to do this in order to get the dynamic pameters from the config file). You don't need to do any modification in this file, just include it inside the launch folder.
    <details>
    <summary>Show the code</summary>
    ```python title="gravity_compensation_launch.py"
        --8<-- "snippets/lab3/gravity_compensation_launch.py"
    ```
    </details>
4. Modify the `CMakeLists.txt` to include the new node
    <details>
    <summary>Show the code</summary>
    ```cmake title="CMakeLists.txt"
        --8<-- "snippets/lab3/CMakeLists_gravity.txt"
    ```
    </details>
4. Once you have done this, the uma_arm_control package should look like this:

    ![workspace_gravity](images/workspace_gravity.png)

5. Now, you can compile the workspace
    ```bash
    cdw
    cb
    ```

### 3.1.2. Launch the controller

To launch the controller you'll need to do the following:

1. Open one terminal and launch the uma_arm_visualization.
2. Open another terminal and launch the controller.
3. Open another terminal and launch the dynamics model

![terminals_gravity](images/terminals_gravity.png)

If you run the `rqt_graph` you should see how the gravity compensation node is getting the `joint_state` and feeding the manipulator with the computed torques

![rqt_gravity](images/rqt_gravity.png)

As a result, now the manipulator stays in the initial position defined by $\mathbf{q}_0 = [45^o, -45^o]$ (note that the initial position is defined by `q0` in the `dynamics_params.yaml`config file).

![result_gravity](images/result_gravity.png)

### 3.1.3. Simulating the force sensor

## 3.2. Linearization by inverse dynamics control

### 3.2.1. Implementation

### 3.2.2. Expected results

## 3.3. Experiments

!!! question
    What happens if the compensation dynamics model is not exactly the same as the manipulator dynamics? 

    1. Try to change the masses `m1`, `m2` and lengths `l1`, `l2` of the links in the `dynamics_params.yaml` (gravity_compensation) file. What are the effects of having incorrent dynamics parameters when launching the gravity compensation controller?
    2. Try the same for the dynamics cancellation. In this case, you can also change the parameters `b1` and `b2`. What are the effects when launching the dynamics cancellation controller?
