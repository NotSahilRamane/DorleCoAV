"""
This a demo of GM-PHD filter simulation for filtering multiple targets.
"""

import numpy as np
import rospy

from GM_PHD_Filter import set_model, GM_PHD_Filter
from av_messages.msg import objects

model = set_model()

# Initialize the initial pruned intensity
pruned_intensity = dict()
pruned_intensity['w'] = []
pruned_intensity['m'] = []
pruned_intensity['P'] = []


# For analysis
pruned_intensity_list = []
estimates_list = []
target_states_list = []
observations_list = []

# Apply the GM-PHD filter for filtering of multiple targets (simulation)
# for i in range(n_scan):
def callback(data):

    # print(i)
    state = np.eye(2, dtype=np.float64)
    Z_k = []
    # Generate target states, actual observations, and clutter
    # target_states = gen_new_states(model, target_states)
    # observations = gen_observations(model, target_states)
    # print("Obs", len(observations))
    # clutter = gen_clutter(model)
    # print("Clutter", len(clutter))
    # Z_k = observations + clutter   # Add clutter to the observations to mimic cluttered environment
    for obj in data.object_detections:
        if obj.status == "Dynamic":
            state[0] = obj.position.x
            state[1] = obj.position.y
            Z_k.append(state)
    
    print("Z_k", len(Z_k))


    # Apply GM-PHD Filter

    # SAHIL GIVE READINGS INSIDE Z_k in the 
    Filter = GM_PHD_Filter(model)
    predicted_intensity = Filter.predict(Z_k, pruned_intensity)
    updated_intensity = Filter.update(Z_k, predicted_intensity)
    pruned_intensity = Filter.prune_and_merge(updated_intensity)
    estimates = Filter.extract_states(pruned_intensity)  # extracting estimates from the pruned intensity this gives
    # better result than extracting them from the updated intensity!

    estimated_states = estimates['m']
    print(estimated_states)
    # Plot ground-truth states, true observations, estimated states and clutter
    # demo_plot(target_states, observations, estimated_states, clutter)

    # Store output intensity and estimates for analysis
    pruned_intensity_list.append(pruned_intensity)
    estimates_list.append(estimates)
    # target_states_list.append(target_states)
    observations_list.append(Z_k)  # observations


def main():

    rospy.init_node('object_tracker')

    while not rospy.is_shutdown():
        rospy.Subscriber('/perception/vision/data', objects,
                         callback=callback) 
        rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
