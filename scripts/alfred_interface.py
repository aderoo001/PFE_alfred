#!/usr/bin/env python

import rospy
import signal
import tkinter as tk

from functools import partial

from std_msgs.msg import Bool


def signal_handler(sig, frame, root):
    root.destroy()


def publish_value(val, publisher, slider):
    if not bool(int(val)):
        slider.configure(troughcolor='red')
    else:
        slider.configure(troughcolor='green')

    msg = Bool()
    msg.data = bool(int(val))
    publisher.publish(msg)


def alfred_interface():
    pub1 = rospy.Publisher('/alfred/allow_vision', Bool, queue_size=10)
    pub2 = rospy.Publisher('/alfred/allow_navigation', Bool, queue_size=10)
    rospy.init_node('alfred_vision', anonymous=True)

    root = tk.Tk()
    root.title("alfred_interface")

    var1 = tk.BooleanVar()
    var2 = tk.BooleanVar()

    label = tk.Label(root, text="Allow vision")
    label.grid(row=2, column=2)

    slider1 = tk.Scale(root, from_=0, to=1, orient=tk.HORIZONTAL, length=60, tickinterval=0,
                       showvalue=False, variable=var1, troughcolor='red',
                       command=lambda value: publish_value(value, pub1, slider1))
    slider1.grid(row=2, column=10)

    label = tk.Label(root, text="Allow navigation")
    label.grid(row=4, column=2)

    slider2 = tk.Scale(root, from_=0, to=1, orient=tk.HORIZONTAL, length=60, tickinterval=0,
                       showvalue=False, variable=var2, troughcolor='red',
                       command=lambda value: publish_value(value, pub2, slider2))
    slider2.grid(row=4, column=10)

    signal_handler_partial = partial(signal_handler, root=root)
    signal.signal(signal.SIGINT, signal_handler_partial)
    signal.signal(signal.SIGTSTP, signal_handler_partial)

    root.mainloop()


if __name__ == '__main__':
    try:
        alfred_interface()
    except rospy.ROSInterruptException:
        pass
