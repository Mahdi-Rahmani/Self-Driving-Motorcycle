import Vortex
import vxatp3
import numpy as np

#Environment Parameters
MAX_POSE = 0.4
SUB_STEPS = 5
MAX_STEPS = 1300
TIME_REWARD = 3
THETA_PENALTY = 1
THETA_DOT_PENALTY = 0.3
THETA_DOT_DOT_PENALTY = 0.05

class env():

    def __init__(self):
        # VxMechanism variable for the mechanism to be loaded.
        self.vxmechanism = None
        self.mechanism = None
        self.interface = None
        self.scene_object = None

        # Define the setup and mechanism file paths
        self.setup_file = 'Resources/Setup.vxc'
        self.content_file = 'Resources/Motorcycle/Motorcycle.vxmechanism'
        self.scene_file = 'Resources/Motorcycle/Motorcycle Scene.vxscene'

        # Create the Vortex Application
        self.application = vxatp3.VxATPConfig.createApplication(self, 'Motorcycle App', self.setup_file)

        # Create a display window
        self.display = Vortex.VxExtensionFactory.create(Vortex.DisplayICD.kExtensionFactoryKey)
        self.display.getInput(Vortex.DisplayICD.kPlacementMode).setValue("Windowed")
        self.display.setName('3D Display')
        self.display.getInput(Vortex.DisplayICD.kPlacement).setValue(Vortex.VxVector4(50, 50, 1280, 720))

        # Initialize Action and Observation Spaces for the NN
        self.max_speed = 8.0
        self.max_pose = 1.0

        high = np.array([1., 1., self.max_speed])
        self.action_space = np.array([-self.max_pose, self.max_pose, (1,)])
        self.observation_space = np.array([-high, high])

    def __del__(self):
        # It is always a good idea to destroy the VxApplication when we are done with it.
        self.application = None

    def reset(self):
        # Initialize Reward and Step Count
        self.current_step = 0
        self.reward = 0

        # The first time we load the mechanism
        if self.vxmechanism is None:
            # Switch to Editing Mode
            vxatp3.VxATPUtils.requestApplicationModeChangeAndWait(self.application, Vortex.kModeEditing)

            # Load mechanism file and get the mechanism interface
            self.vxmechanism = self.application.getSimulationFileManager().loadObject(self.content_file)
            self.mechanism = Vortex.MechanismInterface(self.vxmechanism)

            # Get the RL Interface VHL
            self.interface = self.mechanism.findExtensionByName('RL Interface')

            #scene
            self.scene_object = self.application.getSimulationFileManager().loadObject(self.scene_file)

            # Switch to Simulation Mode
            vxatp3.VxATPUtils.requestApplicationModeChangeAndWait(self.application, Vortex.kModeSimulating)

            # Initialize first key frame
            self.application.update()
            self.keyFrameList = self.application.getContext().getKeyFrameManager().createKeyFrameList("KeyFrameList",
                                                                                                      False)
            self.application.update()

            self.keyFrameList.saveKeyFrame()
            self.waitForNbKeyFrames(1, self.application, self.keyFrameList)
            self.key_frames_array = self.keyFrameList.getKeyFrames()

            # Other times we reset the environment
        else:
            # Switch to Simulation Mode
            vxatp3.VxATPUtils.requestApplicationModeChangeAndWait(self.application, Vortex.kModeSimulating)

            # Load first key frame
            self.keyFrameList.restore(self.key_frames_array[0])
            self.application.update()

        return self._get_obs()

    def waitForNbKeyFrames(self, expectedNbKeyFrames, application, keyFrameList):
        maxNbIter = 100
        nbIter = 0
        while len(keyFrameList.getKeyFrames()) != expectedNbKeyFrames and nbIter < maxNbIter:
            if not application.update():
                break
            ++nbIter

    def step(self, action):  # takes a numpy array as input

        # Apply actions
        self.interface.getInputContainer()['Position'].value = action[0] * MAX_POSE

        # Step the simulation
        for i in range(SUB_STEPS):
            self.application.update()

        # Observations
        obs = self._get_obs()

        # Done flag
        if (self.current_step >= MAX_STEPS) or (abs(obs[0])>60):
            done = True
        else:
            done = False

        # Reward Function
        # Rewarding the Time steps that is passed
        reward = TIME_REWARD * self.current_step**2
        # Penalizing theta
        reward += - THETA_PENALTY * obs[0]**2
        # Penalizing theta_dot
        reward += - THETA_DOT_PENALTY * obs[1]**2
        # Penalizing theta_dot_dot
        reward += - THETA_DOT_DOT_PENALTY * obs[2]**2

        self.current_step += 1

        return obs, reward, done, {}


    def _get_obs(self):
        # Extract values from RL_Interface
        theta = self.interface.getOutputContainer()['theta'].value
        theta_dot = self.interface.getOutputContainer()['thetaDot'].value
        theta_dot_dot = self.interface.getOutputContainer()['thetaDotDot'].value

        return np.array([theta, theta_dot, theta_dot_dot])

    def render(self, active=True):

        # Find current list of displays
        current_displays = self.application.findExtensionsByName('3D Display')

        # If active, add a display and activate Vsync
        if active and len(current_displays) == 0:
            self.application.add(self.display)
            self.application.setSyncMode(Vortex.kSyncSoftwareAndVSync)

        # If not, remove the current display and deactivate Vsync
        elif not active:
            if len(current_displays) == 1:
                self.application.remove(current_displays[0])
            self.application.setSyncMode(Vortex.kSyncNone)
