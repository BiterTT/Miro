# Miro
![Uploading image.png…]()


## Introduction

test.py  is the initial Miro's system structure, containing the ear tail head wheels_move wheels_spin touch 

test2.0.py adds the interface of audio detection module

test3.0.py adds the interface of emotional module and setup the motion of each detected audio speech

test4.0 and test5.0 are in group GitHub repository to show progress in Control touch and audio part. I also help my teammates to integrate the cliff,sonar ,illum and gesture detection into Miro's system structure especially in motion part.

## Methodology

### Touch

![image](https://github.com/user-attachments/assets/549f208c-9b26-430d-836c-32b3f59546b7)

As the touch interaction workflow shown, I developed a mechanism for Miro robot to respond to user  touch inputs by obtaining and processing data to achieve emotional feedback and behavior performance when touched. The system use the range of body touch and head touch flags from the sensors to specify specific action feedback. The sensors value will not be zero once a touch occurs.

### Audio

![image](https://github.com/user-attachments/assets/806ea89f-e39c-46e2-9882-84e0b44e6a0e)


In the voice interaction module,  speech recognition and semantic analysis algorithms enables Miro to understand and respond to the user's voice commands. After configuring the audio class with a list of speech, and the robot will perform the corresponding actions according to the recognized speech symbols. Since the voice will always detect the sound, a separate thread is created to prevent blocking the main thread.

```python
# (just show init here)
class OfflineKeywordListener:
    def __init__(self,controller):
  
        self.robot_name = rospy.get_param("~robot_name", "miro")

        self.pub_tail = self.pub_head

        self.model = Model(r"/home/mima123/vosk-model")
        self.rec = KaldiRecognizer(self.model, 16000)

        self.triggered = False
        
        self.keyword_actions = {
            "hello": lambda:controller.audio_judge("hello"),
            "left": lambda:controller.audio_judge("left"),
            "right": lambda:controller.audio_judge("right"),
            "move": lambda:controller.audio_judge("move"),
            "back": lambda:controller.audio_judge("back"),
            "cycle": lambda:controller.audio_judge("round"),
            "dance": lambda:controller.audio_judge("dance"),
            # "mirror": self.action_shake_head,
            # "hello": self.action_nod,
            # "shake": self.action_shake_tail,
        }
        self.stream = sd.RawInputStream(
            samplerate=16000,
            blocksize=8000,
            dtype='int16',
            channels=1,
            callback=self.audio_callback
        )
```

For the audio part: I add new thread to prevent blocking the main thread

```python
self.Audio = OfflineKeywordListener(self)
self.audio_thread = threading.Thread(target=self.Audio.run)
self.audio_thread.daemon = True  
self.audio_thread.start()
```

### Emotion

In the emotional module, the Miro robot could make different sounds in different mood conditions through modifying valence, arousal, sound level and wakefulness. Then the Miro emits sounds in different mood states, showing the changes in Miro’s mood perception. 

```python
# (just show init here)
class EmotionController:
    def __init__(self, pub_animal_state):
        self.pub_animal_state = pub_animal_state
        # keywords → (valence, arousal, sound_level, wakefulness)
        self.emotion_map = {
            "happy":     (1.0, 0.7, 0.2, 1.0),
            "excited":   (1.0, 1.0, 0.6, 1.0),
            "sad":       (-1.0, 0.3, 0.05, 0.9),
            "angry":     (-1.0, 1.0, 0.6, 1.0),
            "calm":      (0.5, 0.2, 0.1, 1.0),
            "sleepy":    (0.0, 0.1, 0.01, 0.2),
            "neutral":   (0.0, 0.5, 0.1, 1.0),
        }
```

### Whole System Framework Construction

![image](https://github.com/user-attachments/assets/d618066b-b77d-4a58-b721-51b156e1ce12)


For the whole system framework construction, I subscribe essential ROS sensors topics and design system motion control structure and data flow to maintain smooth information transfer and collaboration between the different sensors.  each motion interface contains functions corresponding to sensor control, which can be switched to different control states according to the input parameters, and at the same time, different audio, visual, and touch triggered flags can be used to freely combine the corresponding control activities in order to better perceive the environment and feedback. Since the controller method needs to send control information all the time uninterruptedly. controlling the actuator action for a period of time needs to add the parameters like duration, control time and control flag.

### Certain Period Control


![image](https://github.com/user-attachments/assets/07d2ad80-5dd1-464e-a920-f6f0755406ac)

Since the Miro control method requires continuous sending of control signals, controlling the corresponding sensors to trigger only for a certain period of time needs to be implemented by setting up an algorithm. As shown in the figure, in the loop, through the sensor to get the sensor flag bit such as touch, audio, etc., if the corresponding flag bit is matched, then the **motion_flag** will be set to **True**, after that, the **Motion_time** will be set to **0** and the loop will be carried out, the cycle time will be **Duration**, and the corresponding actuator will be acted in the loop, and the action will be stopped after the loop is exited, and **Motion_time** is set to **0**, then set **Motion_flag** to **False** and wait for the next trigger.

