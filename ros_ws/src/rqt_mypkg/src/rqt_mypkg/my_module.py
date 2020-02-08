#!/bin/usr/env python
import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Connect slider signals to callback
        self._widget.T1.valueChanged.connect(self._slider_callback)
        self._widget.T2.valueChanged.connect(self._slider_callback)
        self._widget.F1_1.valueChanged.connect(self._slider_callback)
        self._widget.F1_2.valueChanged.connect(self._slider_callback)
        self._widget.F2_1.valueChanged.connect(self._slider_callback)
        self._widget.F2_2.valueChanged.connect(self._slider_callback)
        self._widget.F3_1.valueChanged.connect(self._slider_callback)
        self._widget.F3_2.valueChanged.connect(self._slider_callback)

        # Connect button signals to callbacks
        self._widget.save_button.clicked.connect(self._save_callback)
        self._widget.train_button.clicked.connect(self._train_callback)

    def _slider_callback(self, value):
        val = value % 100
        sliderID = int(value / 100)
        print "Value: " + str(val) + " Slider: " + str(sliderID) 

    def _save_callback(self):
        # Get quality from combo box
        quality = self._widget.grasp_list.currentText()

        # Get filename from text input
        fn = self._widget.fn_input.text()

        # Open file
        f = open('/home/grant/ros_ws/src/rqt_mypkg/' + fn + '.txt', 'w')

        # Write robot state and grasp quality to new lines
        if quality == "Bad Grasp":

            # Write 0,1 for bad grasp
            f.write(str([0,1]) + '\n')
        elif quality == "Good Grasp":

            # Write 1,0 for good grasp
            f.write(str([1,0]) + '\n' )
        else:
            pass

        # Close file
        f.close()

        # Reset combo box to "Not Selected"
        self._widget.grasp_list.setCurrentIndex(0)

    def _train_callback(self):
        print "Training ..."

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog