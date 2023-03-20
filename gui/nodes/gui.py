# ! / usr / bin / env python3

# Program to Show how to create a switch
# import kivy module   
import kivy
import rospy
from std_msgs.msg import String

# base Class of your App inherits from the App class.   
# app:always refers to the instance of your application  
from kivy.app import App

# this restrict the kivy version i.e 
# below this kivy version you cannot 
# use the app or software 
kivy.require('1.9.0')

# The Builder is responsible for creating
# a Parser for parsing a kv file
from kivy.lang import Builder

# The Properties classes are used
# when you create an EventDispatcher.
from kivy.properties import NumericProperty

# BoxLayout arranges children in a vertical or horizontal box.
# or help to put the children at the desired location.
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.textinput import TextInput
# he Clock object allows you to
# schedule a function call in the future
from kivy.clock import Clock
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.image import Image
from kivy.uix.button import Button
# Create the .kv file and load it by using Builder

pub = rospy.Publisher('gui_command', String, queue_size=10)

red = [1, 0, 0, 1]
green = [0, 1, 0, 1]
blue = [0, 0, 1, 1]
purple = [1, 0, 1, 1]

# string_list = ['cap drain', 'uncap drain', 'pour jar', 'open door', 'close door', 'open faucet', 'close faucet', 'open fridge', 'close fridge', 'open garbage_can',
#                'close garbage_can', 'open oven', 'close oven', 'open pyrex', 'close pyrex', 'open ziploc', 'close ziploc', 'pour drink', 'throw garbage']
########################################################################

class MyGrid(GridLayout):

    def __init__(self, **kwargs):
        super(MyGrid, self).__init__(**kwargs)
        self.cols = 1

        self.inside = GridLayout()
        self.inside.cols = 2

        self.inside.add_widget(Label(text="Task: "))
        self.task = TextInput(multiline=False)
        self.inside.add_widget(self.task)

        self.add_widget(self.inside)

        self.submit = Button(text="Submit", font_size=40)
        self.submit.bind(on_press=self.pressed)
        self.add_widget(self.submit)

    def pressed(self, instance):
        pub.publish(self.task.text)

class MyApp(App):
    def build(self):
        return MyGrid()
# Run the App
if __name__=="__main__":
    rospy.init_node('gui')
    MyApp().run()


























