import numpy as np

class FourJointedRobotArm:
    def __init__(self, link_lengths):
        self.link_lengths = link_lengths
        self.num_joints = len(link_lengths)
        self.joint_positions = np.zeros((self.num_joints, 3))

    def set_joint_positions(self, joint_positions):
        if len(joint_positions) != self.num_joints:
            raise ValueError("Number of joint positions should match the number of joints")
        self.joint_positions = np.array(joint_positions)

    def calculate_reachability(self, target_position):
        distance_to_target = np.linalg.norm(self.joint_positions[-1] - target_position)
        if distance_to_target <= np.sum(self.link_lengths):
            return True
        else:
            return False

    def move_to_target(self, target_position, tolerance=0, max_iterations=100):
        if not self.calculate_reachability(target_position):
            
            return

        for iteration in range(max_iterations):
            # Forward 
            self.joint_positions[-1] = target_position
            for i in range(self.num_joints - 2, -1, -1):
                self.joint_positions[i] = self.calculate_joint_position(self.joint_positions[i+1], self.joint_positions[i], self.link_lengths[i])

            # Check reachability
            if not self.calculate_reachability(target_position):
                print("Target position became unreachable during iteration", iteration)
                return

            # Backward
            self.joint_positions[0] = self.calculate_joint_position(self.joint_positions[0], self.joint_positions[1], self.link_lengths[0])
            for i in range(1, self.num_joints - 1):
                self.joint_positions[i] = self.calculate_joint_position(self.joint_positions[i], self.joint_positions[i-1], self.link_lengths[i-1])

            # Check convergence
            distance_to_target = np.linalg.norm(self.joint_positions[-1] - target_position)
            if distance_to_target <=tolerance:
                print("Converged after", iteration, "iterations.")
                print("Final joint positions:", self.joint_positions)
                return

            print("Iteration", iteration, "Joint positions:", self.joint_positions)

        print("Reached maximum iterations without convergence.")

    def calculate_joint_position(self, target_position, base_position, link_length):
        vector_to_target = target_position - base_position
        distance_to_target = np.linalg.norm(vector_to_target)
        if distance_to_target > link_length:
            # Adjust target position to be within reachable distance
            target_position = base_position + vector_to_target * (link_length / distance_to_target)
        return target_position

if __name__ == "__main__":
    link_lengths = [23, 15, 9]  # Update with your link lengths
    robot_arm = FourJointedRobotArm(link_lengths)
 
    #PARt - A : Taking Input
    initial_joint_positions = []
    for i in range(robot_arm.num_joints):
        joint_input = input(f"Enter initial joint positions for joint {i+1} (x y z): ")
        initial_joint_positions.append([float(x) for x in joint_input.split()])

    target_input = input("Enter target position (x y z): ")
    target_position = np.array([float(x) for x in target_input.split()])

    robot_arm.set_joint_positions(initial_joint_positions)


    # PART-B: Check reachability of the target position
    if robot_arm.calculate_reachability(target_position):
        print("Yes")
    else:
        print("No")

    # PART-C: Move the arm to the target position
    robot_arm.move_to_target(target_position)
