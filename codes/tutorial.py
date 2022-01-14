#!/usr/bin/env python3.8


from kivy.uix.boxlayout import BoxLayout
import rclpy
from kivymd.app import MDApp
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
import random

class ScatterTextWidget(BoxLayout):
    def change_label_colour(self, *args):
        colour = [random.random() for i in range(3)] + [1]
        label = self.ids['my_label']
        label.color = colour


class TutorialApp(MDApp):

    def build(self):
        return ScatterTextWidget()


if __name__ == '__main__':
    TutorialApp().run()