#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import numpy as np
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QListWidget,
    QFileDialog,
    QLabel,
    QLineEdit,
    QGridLayout,
    QGroupBox,
    QMessageBox,
    QSlider,
)
from PyQt5.QtCore import Qt
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk

from urdf_parser import URDFParser
from urdf_vtk_model import URDFModel




class URDFViewer(QMainWindow):
    """Main application window for URDF viewer"""

    def __init__(self):
        super().__init__()
        self.models = []  # List to store loaded URDF models
        self.init_ui()

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("URDF Viewer")
        self.setGeometry(100, 100, 1200, 800)

        # Create central widget and main layout
        central_widget = QWidget()
        main_layout = QHBoxLayout(central_widget)

        # Create left panel for controls
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        # Create link list
        self.link_list = QListWidget()
        self.link_list.setSelectionMode(QListWidget.SingleSelection)
        self.link_list.itemSelectionChanged.connect(self.on_selection_changed)

        # Create button for opening URDF file
        btn_open = QPushButton("Open URDF")
        btn_open.clicked.connect(self.open_urdf_file)

        # Create transparency controls
        transparency_group = QGroupBox("")
        transparency_layout = QVBoxLayout()

        # Transparency slider
        transparency_layout.addWidget(QLabel("Transparency (All Links):"))
        self.transparency_slider = QSlider(Qt.Horizontal)
        self.transparency_slider.setMinimum(0)
        self.transparency_slider.setMaximum(100)
        self.transparency_slider.setValue(100)  # Default to fully opaque
        self.transparency_slider.setTickPosition(QSlider.TicksBelow)
        self.transparency_slider.setTickInterval(10)
        self.transparency_slider.valueChanged.connect(self.apply_transparency)
        transparency_layout.addWidget(self.transparency_slider)

        transparency_group.setLayout(transparency_layout)

        # Add widgets to left panel
        left_layout.addWidget(QLabel("Robot Links:"))
        left_layout.addWidget(self.link_list)
        left_layout.addWidget(btn_open)
        left_layout.addWidget(transparency_group)
        left_layout.addStretch()

        # Set fixed width for left panel
        left_panel.setFixedWidth(300)

        # Create VTK widget for 3D visualization
        self.vtk_widget = QVTKRenderWindowInteractor()

        # Add panels to main layout
        main_layout.addWidget(left_panel)
        main_layout.addWidget(self.vtk_widget, 1)

        # Set central widget
        self.setCentralWidget(central_widget)

        # Set up VTK rendering
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.9, 0.9, 0.9)  # Dark gray background
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)

        # Set up interactor
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()

        # Configure interactor style for rotation with left mouse button
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.interactor.SetInteractorStyle(style)

        # Initialize interactor
        self.interactor.Initialize()

        # Add world axes for reference
        # self.add_world_axes()

        # Start the interactor
        self.interactor.Start()

    def add_world_axes(self):
        """Add world coordinate axes to the scene"""
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.1, 0.1, 0.1)  # Set the length of the axes
        axes.SetShaftType(0)
        axes.SetAxisLabels(0)  # Not Show labels
        axes.SetCylinderRadius(0.02)

        # Add the axes to the renderer
        self.renderer.AddActor(axes)

    def open_urdf_file(self):
        """Open a URDF file and visualize the robot"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Open URDF File", "", "URDF Files (*.urdf *.xml)"
        )

        if filename:
            # Clear previous models
            self.clear_models()

            # Parse the URDF file
            try:
                parser = URDFParser(filename)
                (link_names,
                link_mesh_files,
                link_mesh_transformations,
                link_frames,
                link_colors,
                joint_names,
                joint_frames,
                joint_types,
                joint_axes,
                joint_parent_links,
                joint_child_links) = parser.get_robot_info()
                
                chains, _ = parser.get_chain_info()
                mdh_frames = parser.get_mdh_frames(chains[0])

                # Create models for each link
                for i in range(len(link_names)):
                    self.add_urdf_model(
                        link_names[i],
                        link_mesh_files[i],
                        link_mesh_transformations[i],
                        mdh_frames[i], # link_frames[i],
                        link_colors[i],
                    )

                # Reset camera to show all actors
                self.renderer.ResetCamera()
                self.vtk_widget.GetRenderWindow().Render()

            except Exception as e:
                QMessageBox.critical(
                    self, "Error", f"Failed to load URDF file: {str(e)}"
                )

    def add_urdf_model(self, name, mesh_file, mesh_transform, frame, color):
        """Add a URDF model to the scene"""
        try:
            # Create a new URDF model
            model = URDFModel(name, mesh_file, mesh_transform, frame, color)

            # Apply current transparency setting to the new model
            if hasattr(self, "transparency_slider"):
                transparency = self.transparency_slider.value() / 100.0
                model.set_transparency(transparency)

            # Add the model to our list
            self.models.append(model)

            # Add the actor to the renderer
            self.renderer.AddActor(model.actor)

            # Add the axes actor to the renderer
            self.renderer.AddActor(model.axes_actor)

            # Add the model name to the list widget
            self.link_list.addItem(model.name)

        except Exception as e:
            QMessageBox.warning(
                self, "Warning", f"Failed to load model {name}: {str(e)}"
            )

    def clear_models(self):
        """Clear all models from the scene"""
        # Remove all actors from the renderer
        for model in self.models:
            self.renderer.RemoveActor(model.actor)
            self.renderer.RemoveActor(model.axes_actor)

        # Clear the models list
        self.models = []

        # Clear the list widget
        self.link_list.clear()

        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    def on_selection_changed(self):
        """Handle selection change in the link list"""
        selected_items = self.link_list.selectedItems()

        # Unhighlight all models first
        for model in self.models:
            model.unhighlight()

        if not selected_items:
            # Update the rendering if no selection
            self.vtk_widget.GetRenderWindow().Render()
            return

        # Get the selected model
        selected_index = self.link_list.row(selected_items[0])
        model = self.models[selected_index]

        # Highlight the selected model
        model.highlight()

        # Update the rendering to show the highlighting
        self.vtk_widget.GetRenderWindow().Render()

    def apply_transparency(self):
        """Apply transparency to all loaded models"""
        # Get transparency value from slider (convert from 0-100 to 0.0-1.0)
        transparency = self.transparency_slider.value() / 100.0

        # Apply the transparency to all models
        for model in self.models:
            model.set_transparency(transparency)

        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    def closeEvent(self, event):
        """Handle window close event"""
        # Properly clean up the VTK widget
        self.vtk_widget.GetRenderWindow().Finalize()
        self.vtk_widget.close()
        event.accept()


def main():
    """Main function to run the application"""
    app = QApplication(sys.argv)
    viewer = URDFViewer()
    viewer.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
