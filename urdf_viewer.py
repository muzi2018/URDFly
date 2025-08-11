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
    QTreeWidget,
    QTreeWidgetItem,
    QCheckBox,
    QDialog,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
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
        self.chains = []  # List to store kinematic chains
        self.mdh_frames_actors = []  # List to store MDH frame actors
        self.selected_chain = None  # Currently selected chain
        self.current_urdf_file = None  # Path to the currently loaded URDF file
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

        # Create chain tree widget instead of link list
        self.chain_tree = QTreeWidget()
        self.chain_tree.setHeaderLabel("Robot Chains and Links")
        self.chain_tree.setSelectionMode(QTreeWidget.SingleSelection)
        self.chain_tree.itemSelectionChanged.connect(self.on_tree_selection_changed)

        # Create button for opening URDF file
        btn_open = QPushButton("Open URDF")
        btn_open.clicked.connect(self.open_urdf_file)

        # Create MDH button
        btn_mdh = QPushButton("Show MDH Parameters")
        btn_mdh.clicked.connect(self.show_mdh_parameters)

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

        # Create checkboxes for frame visibility
        visibility_group = QGroupBox("Frame Visibility")
        visibility_layout = QVBoxLayout()
        
        self.cb_link_frames = QCheckBox("Show Link Frames")
        self.cb_link_frames.setChecked(True)
        self.cb_link_frames.stateChanged.connect(self.toggle_link_frames)
        
        self.cb_mdh_frames = QCheckBox("Show MDH Frames")
        self.cb_mdh_frames.setChecked(False)
        self.cb_mdh_frames.stateChanged.connect(self.toggle_mdh_frames)
        
        visibility_layout.addWidget(self.cb_link_frames)
        visibility_layout.addWidget(self.cb_mdh_frames)
        visibility_group.setLayout(visibility_layout)

        # Add widgets to left panel
        left_layout.addWidget(QLabel("Robot Structure:"))
        left_layout.addWidget(self.chain_tree)
        left_layout.addWidget(btn_open)
        left_layout.addWidget(btn_mdh)
        left_layout.addWidget(transparency_group)
        left_layout.addWidget(visibility_group)
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
                
                # Get robot info for visualization
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
                
                # Get chain information
                self.chains, trees = parser.get_chain_info()
                
                # Create models for each link
                for i in range(len(link_names)):
                    self.add_urdf_model(
                        link_names[i],
                        link_mesh_files[i],
                        link_mesh_transformations[i],
                        link_frames[i],
                        link_colors[i],
                    )
                
                # Populate the chain tree
                self.populate_chain_tree()

                # Reset camera to show all actors
                self.renderer.ResetCamera()
                self.vtk_widget.GetRenderWindow().Render()
                
                # Store the current URDF file path only after successful loading
                self.current_urdf_file = filename

            except Exception as e:
                QMessageBox.critical(
                    self, "Error", f"Failed to load URDF file: {str(e)}"
                )

    def add_urdf_model(self, name, mesh_file, mesh_transform, frame, color):
        """Add a URDF model to the scene"""
        try:
            # Create a new URDF model with axis text (using link name as the text)
            model = URDFModel(name, mesh_file, mesh_transform, frame, color, axis_text=name)

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
            
            # Add the text actor to the renderer if it exists
            if model.text_actor is not None:
                self.renderer.AddActor(model.text_actor)
            
            # Set initial visibility based on checkbox
            if hasattr(self, "cb_link_frames"):
                model.axes_actor.SetVisibility(self.cb_link_frames.isChecked())
                if model.text_actor is not None:
                    model.text_actor.SetVisibility(self.cb_link_frames.isChecked())

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
            if hasattr(model, 'text_actor') and model.text_actor is not None:
                self.renderer.RemoveActor(model.text_actor)

        # Clear the models list
        self.models = []

        # Clear the tree widget
        self.chain_tree.clear()
        
        # Clear MDH frames
        for actor in self.mdh_frames_actors:
            self.renderer.RemoveActor(actor)
        self.mdh_frames_actors = []
        
        # Reset selected chain and current URDF file
        self.selected_chain = None
        self.current_urdf_file = None

        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    def populate_chain_tree(self):
        """Populate the chain tree with chains and links"""
        self.chain_tree.clear()
        
        # Create a dictionary to map link names to models
        link_to_model = {model.name: model for model in self.models}
        
        # Add each chain as a top-level item
        for i, chain in enumerate(self.chains):
            chain_item = QTreeWidgetItem(self.chain_tree)
            chain_item.setText(0, f"Chain {i+1}: {chain['name']}")
            chain_item.setData(0, Qt.UserRole, i)  # Store chain index
            
            # Add links as child items
            for link_name in chain['link_names']:
                link_item = QTreeWidgetItem(chain_item)
                link_item.setText(0, link_name)
                link_item.setData(0, Qt.UserRole, link_name)  # Store link name
        
        # Expand all items
        self.chain_tree.expandAll()

    def on_tree_selection_changed(self):
        """Handle selection change in the chain tree"""
        selected_items = self.chain_tree.selectedItems()
        
        # Unhighlight all models first
        for model in self.models:
            model.unhighlight()
        
        # Reset selected chain
        self.selected_chain = None
        
        if not selected_items:
            # Update the rendering if no selection
            self.vtk_widget.GetRenderWindow().Render()
            return
        
        selected_item = selected_items[0]
        parent = selected_item.parent()
        
        if parent is None:
            # Chain is selected
            chain_index = selected_item.data(0, Qt.UserRole)
            self.selected_chain = self.chains[chain_index]
            
            # Highlight all links in the chain
            for link_name in self.selected_chain['link_names']:
                for model in self.models:
                    if model.name == link_name:
                        model.highlight()
        else:
            # Link is selected
            link_name = selected_item.data(0, Qt.UserRole)
            
            # Highlight the selected link
            for model in self.models:
                if model.name == link_name:
                    model.highlight()
        
        # Update the rendering to show the highlighting
        self.vtk_widget.GetRenderWindow().Render()

    def toggle_link_frames(self, state):
        """Toggle visibility of link frames and their text labels"""
        visible = state == Qt.Checked
        
        for model in self.models:
            model.axes_actor.SetVisibility(visible)
            if hasattr(model, 'text_actor') and model.text_actor is not None:
                model.text_actor.SetVisibility(visible)
        
        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    def toggle_mdh_frames(self, state):
        """Toggle visibility of MDH frames"""
        visible = state == Qt.Checked
        
        # If MDH frames are not created yet and we want to show them
        if visible and not self.mdh_frames_actors and self.selected_chain and self.current_urdf_file:
            self.create_mdh_frames(self.selected_chain)
        elif visible and (not self.selected_chain or not self.current_urdf_file):
            QMessageBox.warning(
                self, "Warning", "Please select a chain and load a URDF file first."
            )
            self.cb_mdh_frames.setChecked(False)
            return
        
        # Toggle visibility of existing MDH frames
        for actor in self.mdh_frames_actors:
            actor.SetVisibility(visible)
        
        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    def create_mdh_frames(self, chain):
        """Create MDH frame actors for the selected chain"""
        # Clear existing MDH frames
        for actor in self.mdh_frames_actors:
            self.renderer.RemoveActor(actor)
        self.mdh_frames_actors = []
        
        # Get MDH frames from the parser
        parser = URDFParser(self.current_urdf_file)  # Use the current URDF file
        mdh_frames = parser.get_mdh_frames(chain)
        
        # Create axes actors for each MDH frame
        for frame in mdh_frames:
            axes = vtk.vtkAxesActor()
            axes.SetTotalLength(0.05, 0.05, 0.05)  # Set the length of the axes
            axes.SetShaftType(0)
            axes.SetAxisLabels(0)
            axes.SetCylinderRadius(0.01)
            
            # Create transform
            vtk_transform = vtk.vtkTransform()
            vtk_transform.SetMatrix(frame.flatten())
            axes.SetUserTransform(vtk_transform)
            
            # Add to renderer
            self.renderer.AddActor(axes)
            self.mdh_frames_actors.append(axes)
            
            # Set initial visibility based on checkbox
            axes.SetVisibility(self.cb_mdh_frames.isChecked())
        
        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    def show_mdh_parameters(self):
        """Show MDH parameters in a dialog"""
        if not self.selected_chain:
            QMessageBox.warning(
                self, "Warning", "Please select a chain first to view MDH parameters."
            )
            return
        
        if not self.current_urdf_file:
            QMessageBox.warning(
                self, "Warning", "Please load a URDF file first."
            )
            return
        
        # Create MDH parameter dialog
        dialog = QDialog(self)
        dialog.setWindowTitle(f"MDH Parameters - {self.selected_chain['name']}")
        dialog.setMinimumWidth(500)
        dialog.setMinimumHeight(300)
        
        # Create layout
        layout = QVBoxLayout(dialog)
        
        # Create table
        table = QTableWidget()
        table.setColumnCount(5)  # Joint, theta, d, a, alpha
        table.setHorizontalHeaderLabels(["Joint", "θ (rad)", "d", "a", "α (rad)"])
        
        # Get MDH parameters
        parser = URDFParser(self.current_urdf_file)  # Use the current URDF file
        _, _, _, mdh_parameters = parser.get_mdh_parameters(self.selected_chain)
        
        # Set row count
        table.setRowCount(len(mdh_parameters))
        
        # Fill table with MDH parameters
        for i, params in enumerate(mdh_parameters):
            # Joint name (use link name or joint index)
            joint_item = QTableWidgetItem(f"Joint {i+1}")
            table.setItem(i, 0, joint_item)
            
            # MDH parameters: theta, d, a, alpha
            for j, param in enumerate(params):
                param_item = QTableWidgetItem(f"{param:.4f}")
                table.setItem(i, j+1, param_item)
        
        # Resize columns to content
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        # Add table to layout
        layout.addWidget(table)
        
        # Add close button
        btn_close = QPushButton("Close")
        btn_close.clicked.connect(dialog.accept)
        layout.addWidget(btn_close)
        
        # Show dialog
        dialog.exec_()

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
