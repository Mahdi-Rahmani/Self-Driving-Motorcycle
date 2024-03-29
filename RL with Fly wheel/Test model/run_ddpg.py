import tensorflow as tf
import numpy as np
from environment_test import env

LOAD_MODEL_ACTOR = 'models\ddpg_motor_vortex_Actor__144.58avg__1691573897.model'
#LOAD_MODEL_ACTOR = 'models\ddpg_motor_vortex_Actor__149.09avg__1691536608.model'
#LOAD_MODEL_ACTOR = 'models\ddpg_motor_vortex_Actor__259.15avg__1691562440.model'
#LOAD_MODEL_ACTOR = 'models\ddpg_motor_vortex_Actor__154.67avg__1691558061.model'
#LOAD_MODEL_ACTOR = 'models\ddpg_motor_vortex_Actor__149.09avg__1691536608.model'
#LOAD_MODEL_ACTOR = 'models\ddpg_motor_vortex_Actor__124.18avg__1691536569.model'
actor_model = tf.keras.models.load_model(LOAD_MODEL_ACTOR)


env = env()
env.render(True)

prev_state = env.reset()

episodic_reward = 0

# For HUD
env.interface.getInputContainer()['episode'].value = 'N/A'

while True:

    tf_prev_state = tf.expand_dims(tf.convert_to_tensor(prev_state), 0)
    action = [tf.squeeze(actor_model(tf_prev_state)).numpy()]
    # Recieve state and reward from environment.
    state, reward, done, info = env.step(action)

    episodic_reward += reward

    # For HUD
    env.interface.getInputContainer()['reward'].value = str(int(episodic_reward))

    # End this episode when `done` is True
    if done:
        break

    prev_state = state

print(episodic_reward)

# Make sure the environment is deleted first. This will also cleanup the Vortex application.
del env
