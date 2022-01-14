from kivy.app import App
from kivy.lang import Builder
from kivy.uix.floatlayout import FloatLayout

kv_text = """\
#:kivy 1.7.2
<MyWidget>
    Widget:
        on_parent: menupanel.dismiss()

    DropDown:
        id: menupanel
        on_select: button_opendd.text = '{}'.format(args[1])

        Button:
            text: 'A'
            size_hint_y: None
            height: 30
            on_release: menupanel.select('A')

        Button:
            text: 'B'
            size_hint_y: None
            height: 30
            on_release: menupanel.select('B')

    Button:
        id: button_opendd
        pos_hint: {"top":1, "right":1}
        text: 'Press'
        on_release: menupanel.open(self)
        size_hint_y: None
        height: 40
"""

class MyWidget(FloatLayout):
    def __init__(self, **kwargs):
        super(MyWidget, self).__init__(**kwargs)

class MyWidgetApp(App):
    def build(self):
        return MyWidget()

def main():
    Builder.load_string(kv_text)
    app = MyWidgetApp()
    app.run()

if __name__ == '__main__':
    main()