import logging
import os
import vtk
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
import slicer, logging, sitkUtils

import SimpleITK as sitk
import qt
import time
import numpy as np
import math


from navigationLib.niiCook import niiCook
from navigationLib.simulationFunction import makeSimulation

from qt import QTimer
#
# navigation
#

class navigation(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = "navigation"  # TODO: make this more human readable by adding spaces
        self.parent.categories = ["Neumous"]  # TODO: set categories (folders where the module shows up in the module selector)
        self.parent.dependencies = []  # TODO: add here list of module names that this module requires
        self.parent.contributors = ["John Doe (AnyWare Corp.)"]  # TODO: replace with "Firstname Lastname (Organization)"
        # TODO: update with short description of the module and a link to online module documentation
        self.parent.helpText = """
This is an example of scripted loadable module bundled in an extension.
See more information in <a href="https://github.com/organization/projectname#navigation">module documentation</a>.
"""
        # TODO: replace with organization, grant and thanks
        self.parent.acknowledgementText = """
This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc., Andras Lasso, PerkLab,
and Steve Pieper, Isomics, Inc. and was partially funded by NIH grant 3P41RR013218-12S1.
"""

        # Additional initialization step after application startup is complete
        # slicer.app.connect("startupCompleted()", registerSampleData)



class navigationWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
    """Uses ScriptedLoadableModuleWidget base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent=None):
        """
        Called when the user opens the module the first time and the widget is initialized.
        """
        ScriptedLoadableModuleWidget.__init__(self, parent)
        VTKObservationMixin.__init__(self)  # needed for parameter node observation
        self.logic = None
        self._parameterNode = None
        self._updatingGUIFromParameterNode = False

    def setup(self):
        """
        Called when the user opens the module the first time and the widget is initialized.
        """
        ScriptedLoadableModuleWidget.setup(self)

        # Load widget from .ui file (created by Qt Designer).
        # Additional widgets can be instantiated manually and added to self.layout.
        uiWidget = slicer.util.loadUI(self.resourcePath('UI/navigation.ui'))
        self.layout.addWidget(uiWidget)
        self.ui = slicer.util.childWidgetVariables(uiWidget)

        # https://www.biew.co.kr/entry/Visual-Studio-CodeVS-Code-%EC%9C%A0%EC%9A%A9%ED%95%9C-%EB%8B%A8%EC%B6%95%ED%82%A4-%EC%82%AC%EC%9A%A9-%EB%B0%A9%EB%B2%95Set scene in MRML widgets. Make sure that in Qt designer the top-level qMRMLWidget's
        # "mrmlSceneChanged(vtkMRMLScene*)" signal in is connected to each MRML widget's.
        # "setMRMLScene(vtkMRMLScene*)" slot.
        uiWidget.setMRMLScene(slicer.mrmlScene)

        ################################################################################################
        # camera setting 
        myFont = qt.QFont()
        myFont.setBold(True)
        
        self.ui.referenceLabel.setStyleSheet("color: #FA8072; width: 15px; ")
        self.ui.referenceLabel.setFont(myFont)
        
        self.ui.transducerLabel.setStyleSheet("color: #FA8072; width: 15px; ")
        self.ui.transducerLabel.setFont(myFont)

        self.ui.stylusLabel.setStyleSheet("color: #FA8072; width: 15px; ")
        self.ui.stylusLabel.setFont(myFont)

        self.ui.cameraButton.connect('clicked(bool)', self.onSetCameraButton)

        ################################################################################################
        # Pre processing

        self.ui.findVolumeButton.connect('clicked(bool)', self.onSetFindVolumeButton)

        self.ui.inputSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.updateParameterNodeFromGUI)
        self.ui.preprocessButton.connect('clicked(bool)', self.onSetPreprocessButton)
        
        self.ui.makeTransducerButton.connect('clicked(bool)', self.onSetMakeTransducerButton)
        self.simul = makeSimulation()
                
        ################################################################################################
        # Point setting 
        
        self.markupsLogic = slicer.modules.markups.logic()
        
        self.fromMarkupsNode = slicer.mrmlScene.GetNodeByID(self.markupsLogic.AddNewFiducialNode())
        self.fromMarkupsNode.SetName("From")

        self.toMarkupsNode = slicer.mrmlScene.GetNodeByID(self.markupsLogic.AddNewFiducialNode())
        self.toMarkupsNode.SetName("To")

        self.fromFiducial = self.ui.FromMarkupsWidget
        self.fromFiducial.setMRMLScene(slicer.mrmlScene)
        self.fromFiducial.setCurrentNode(self.fromMarkupsNode)
               
        self.toFiducial = self.ui.ToMarkupsWidget
        self.toFiducial.setMRMLScene(slicer.mrmlScene)
        self.toFiducial.setCurrentNode(self.toMarkupsNode)
        
        defaultColor = qt.QColor(0, 0, 255)
        self.toFiducial.setNodeColor(defaultColor)
        
        self.ui.getPointsButton.connect('clicked(bool)', self.onSetGetPointButton)
        
        ################################################################################################
        # Registration
        
        self.ui.registrationButton.connect('clicked(bool)', self.onSetRegistrationButton)
        self.ui.trackingCheckBox.clicked.connect(self.onSetTrackingCheckBox)
        self.ui.renderingCheckBox.clicked.connect(self.onSetRenderingCheckBox)
        
        ################################################################################################
        # Surf Point setting 
        
        self.markupsLogic = slicer.modules.markups.logic()
        
        self.surfFromMarkupsNode = slicer.mrmlScene.GetNodeByID(self.markupsLogic.AddNewFiducialNode())
        self.surfFromMarkupsNode.SetName("surface regi From")

        self.surfFromFiducial = self.ui.surfFromMarkupsWidget
        self.surfFromFiducial.setMRMLScene(slicer.mrmlScene)
        self.surfFromFiducial.setCurrentNode(self.surfFromMarkupsNode)
        
        defaultColor = qt.QColor(255, 100, 0)
        self.surfFromFiducial.setNodeColor(defaultColor)
        
        self.ui.getPointsCheckBox.clicked.connect(self.onSetGetPointCloudButton)
        
        ################################################################################################
        # Surf regi setting
        self.ui.surfRegistrationButton.connect('clicked(bool)', self.onSetSurfRegistrationButton)
        

    def updateGUIFromParameterNode(self, caller=None, event=None):
        """
        This method is called whenever parameter node is changed.
        The module GUI is updated to show the current state of the parameter node.
        """
        if self._parameterNode is None or self._updatingGUIFromParameterNode:
            return

        # Make sure GUI changes do not call updateParameterNodeFromGUI (it could cause infinite loop)
        self._updatingGUIFromParameterNode = True

        # Update node selectors and sliders
        self.ui.inputSelector.setCurrentNode(self._parameterNode.GetNodeReference("InputVolume"))
        self.ui.cameraButton.setCurrentNode(self._parameterNode.GetNodeReference("SetCamera"))

        # All the GUI updates are done
        self._updatingGUIFromParameterNode = False

    def updateParameterNodeFromGUI(self, caller=None, event=None):
        """
        This method is called when the user makes any change in the GUI.
        The changes are saved into the parameter node (so that they are restored when the scene is saved and loaded).
        """
        if self._parameterNode is None or self._updatingGUIFromParameterNode:
            return

        self._parameterNode.SetNodeReferenceID("InputVolume", self.ui.inputSelector.currentNodeID)
        self._parameterNode.SetNodeReferenceID("SetCamera", self.ui.cameraButton.currentNodeID)

    def onSetFindVolumeButton(self):
        fname = qt.QFileDialog.getOpenFileName()
        cook = niiCook()
        cook.readSavedFile(fname)
        image = cook.itkImage

        self.volumeNode = sitkUtils.PushVolumeToSlicer(image)
        self.volumeNode.SetName("Volume")
        

    def onSetPreprocessButton(self):

        masterVolumeNode = self.ui.inputSelector.currentNode()

        simpleitk_image = sitkUtils.PullVolumeFromSlicer(masterVolumeNode)

        otsuFilter = sitk.OtsuThresholdImageFilter()
        otsuFilter.Execute(simpleitk_image)
        otsuThreshold = otsuFilter.GetThreshold()

        minValue  = otsuThreshold
        maxValue  = 5000

        segmentationNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLSegmentationNode")
        segmentationNode.SetName('Segmentation')
        #segmentationNode = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLSegmentationNode")
        #segmentationNode = slicer.util.getNode('Segmentation_1')
        segmentationNode.CreateDefaultDisplayNodes() # only needed for display
        segmentationNode.SetReferenceImageGeometryParameterFromVolumeNode(masterVolumeNode)

        # Create temporary segment editor to get access to effects
        segmentEditorWidget = slicer.qMRMLSegmentEditorWidget()
        segmentEditorWidget.setMRMLScene(slicer.mrmlScene)
        segmentEditorNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLSegmentEditorNode")

        # Do masking
        addedSegmentID = segmentationNode.GetSegmentation().AddEmptySegment("head")
        segmentEditorNode.SetMaskSegmentID(addedSegmentID)
        segmentEditorNode.SetOverwriteMode(slicer.vtkMRMLSegmentEditorNode.OverwriteAllSegments)
        # segmentEditorNode.SetMaskMode(slicer.vtkMRMLSegmentEditorNode.PaintAllowedInsideSingleSegment)

        segmentEditorWidget.setMRMLSegmentEditorNode(segmentEditorNode)
        segmentEditorWidget.setSegmentationNode(segmentationNode)
        segmentEditorWidget.setMasterVolumeNode(masterVolumeNode)
        segmentationDisplayNode=segmentationNode.GetDisplayNode()
        segmentation=segmentationNode.GetSegmentation()

        slicer.app.processEvents()
        segmentEditorWidget.setActiveEffectByName("Threshold")
        effect = segmentEditorWidget.activeEffect()
        effect.setParameter("MinimumThreshold", minValue)
        effect.setParameter("MaximumThreshold", maxValue)
        effect.self().onApply()

        slicer.app.processEvents()
        segmentEditorWidget.setActiveEffectByName("Islands")
        effect = segmentEditorWidget.activeEffect()
        effect.setParameter("Operation", "KEEP_LARGEST_ISLAND")
        effect.self().onApply()

        # slicer.app.processEvents()
        # segmentEditorWidget.setActiveEffectByName("Wrap Solidify")
        # effect = segmentEditorWidget.activeEffect()
        # effect.setParameter("region", "outerSurface")
        # # effect.setParameter("carveHolesInOuterSurface", True)
        # # effect.setParameter("carveHolesInOuterSurfaceDiameter", 80)
        # effect.setParameter("outputType", "segment")
        # #effect.setParameter("remeshOversampling", 0.3)
        # effect.self().onApply()
        
        slicer.app.processEvents()
        segmentEditorWidget.setActiveEffectByName("Margin")
        effect = segmentEditorWidget.activeEffect()
        effect.setParameter("MarginSizeMm", 0.1)
        effect.self().onApply()

        shNode = slicer.mrmlScene.GetSubjectHierarchyNode()
        exportFolderItemId = shNode.CreateFolderItem(shNode.GetSceneItemID(), "Segments")
        slicer.modules.segmentations.logic().ExportAllSegmentsToModels(segmentationNode, exportFolderItemId)


    def onSetMakeTransducerButton(self):
        
        image = sitk.Image(128, 128, 128, sitk.sitkInt16)
        image.SetOrigin([-64, -64, -64])
        
        self.simul.ROC = 100  
        self.simul.width = 85  
        self.simul.focal_length = 92

        self.simul.preprocessing(image, [0,0,10])
        self.simul.make_transducer([0,0,0])

        focalMarkupsNode = slicer.mrmlScene.GetNodeByID(self.markupsLogic.AddNewFiducialNode())
        focalMarkupsNode.SetName("Focal Point")
        focalMarkupsNode.AddControlPoint([0,0,self.simul.focal_length])
                
        transducerNode = sitkUtils.PushVolumeToSlicer(self.simul.trans_itk)
        transducerNode.SetName("Transducer")

        volRenLogic = slicer.modules.volumerendering.logic()
        DisplayNode = volRenLogic.CreateDefaultVolumeRenderingNodes(transducerNode)
        DisplayNode.SetVisibility3D(True)

        transducerNode.SetAndObserveTransformNodeID(self.TranCenterToRefernceNode.GetID())        
        focalMarkupsNode.SetAndObserveTransformNodeID(self.TranCenterToRefernceNode.GetID())
        self.focalMarkupsNode = focalMarkupsNode
                
    def onSetCameraButton(self):
        
        scriptPath = os.path.dirname(os.path.abspath(__file__))
        print(os.path.join(scriptPath, 'model\\NeedleModel.stl'))
        needleNode = slicer.util.loadModel(os.path.join(scriptPath, 'Resources\\model\\NeedleModel.stl'))

        igtNode = slicer.vtkMRMLIGTLConnectorNode()
        slicer.mrmlScene.AddNode(igtNode)
        igtNode.SetName('igtNode')
        igtNode.SetTypeClient('localhost', 18944)
        igtNode.Start()

        self.ReferenceToTrackerNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", "ReferenceToTracker")
        self.TranCenterToRefernceNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", "TranCenterToReferenc")
        self.StylusTipToReferenceNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", "StylusTipToReference")

        needleNode.SetAndObserveTransformNodeID(self.StylusTipToReferenceNode.GetID())

        self.lastMatrix = [np.zeros((4, 4)), np.zeros((4, 4)), np.zeros((4, 4))] 
        transformMatrix = vtk.vtkMatrix4x4()

        def checkTransformNode(node, lastMatrix):
            
            name = node.GetName()
            node.GetMatrixTransformToWorld(transformMatrix)
            matrix_arr_origin = np.zeros((4, 4))


            for i in range(0, 4):
                for j in range(0, 4):
                    matrix_arr_origin[i, j] = (transformMatrix.GetElement(i, j))
            
            if np.array_equal(lastMatrix, matrix_arr_origin):
                if name== "ReferenceToTracker":
                    self.ui.referenceLabel.setText("Reference Off")
                    self.ui.referenceLabel.setStyleSheet("color: #FA8072; width: 15px; ")
                elif name== "TranCenterToReferenc":
                    self.ui.transducerLabel.setText("Transducer Off")
                    self.ui.transducerLabel.setStyleSheet("color: #FA8072; width: 15px; ")
                elif name== "StylusTipToReference":
                    self.ui.stylusLabel.setText("Stylus Off")                    
                    self.ui.stylusLabel.setStyleSheet("color: #FA8072; width: 15px; ")
            else:
                if name== "ReferenceToTracker":
                    self.ui.referenceLabel.setText("Reference On")
                    self.ui.referenceLabel.setStyleSheet("color: #4D69E8; width: 15px; ")
                elif name== "TranCenterToReferenc":
                    self.ui.transducerLabel.setText("Transducer On")
                    self.ui.transducerLabel.setStyleSheet("color: #4D69E8; width: 15px; ")
                elif name== "StylusTipToReference":
                    self.ui.stylusLabel.setText("Stylus On")
                    self.ui.stylusLabel.setStyleSheet("color: #4D69E8; width: 15px; ")
                    
            # save current matrix
            if name== "ReferenceToTracker":
                self.lastMatrix[0] = matrix_arr_origin
            elif name== "TranCenterToReferenc":
                self.lastMatrix[1] = matrix_arr_origin
            elif name== "StylusTipToReference":
                self.lastMatrix[2] = matrix_arr_origin

        #ReferenceToTrackerNode.AddObserver(slicer.vtkMRMLWatchdogNode.TransformModifiedEvent, lambda caller, event: checkTransformNode(caller, event, self.lastMatrix[0]))

        timer = QTimer()
        timer.timeout.connect(lambda: checkTransformNode(self.ReferenceToTrackerNode, self.lastMatrix[0]))
        timer.timeout.connect(lambda: checkTransformNode(self.TranCenterToRefernceNode, self.lastMatrix[1]))
        timer.timeout.connect(lambda: checkTransformNode(self.StylusTipToReferenceNode, self.lastMatrix[2]))
        timer.start(100)
        
        # start event loop 
        while True:
            slicer.app.processEvents() 

    def onSetGetPointButton(self):
        
        transformMatrix = vtk.vtkMatrix4x4()
        
        StylusTipToReferenceNode = slicer.util.getNode("StylusTipToReference")       
        StylusTipToReferenceNode.GetMatrixTransformToWorld(transformMatrix)
        
        matrix_arr = np.zeros((4, 4))
        for i in range(0, 4):
            for j in range(0, 4):
                matrix_arr[i, j] = (transformMatrix.GetElement(i, j))
                
        point = matrix_arr[:3,-1]
        self.fromMarkupsNode.AddControlPoint(point)
        
    def onSetRegistrationButton(self):
                
        self.ReferenceToImageNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", "ReferenceToImage")
        
        self.pointRegistrationNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLFiducialRegistrationWizardNode", "pointRegistration")
        self.pointRegistrationNode.SetAndObserveFromFiducialListNodeId(self.fromMarkupsNode.GetID())
        self.pointRegistrationNode.SetAndObserveToFiducialListNodeId(self.toMarkupsNode.GetID())
        self.pointRegistrationNode.SetOutputTransformNodeId(self.ReferenceToImageNode.GetID())
        self.pointRegistrationNode.SetRegistrationModeToRigid()
        
        slicer.app.processEvents()
        
        error = np.round(self.pointRegistrationNode.GetCalibrationError(), 2)
        
        self.ui.StatusLabel.setText("Status: "+str(error)+" mm")

        self.fromMarkupsNode.SetAndObserveTransformNodeID(self.ReferenceToImageNode.GetID())
        self.TranCenterToRefernceNode.SetAndObserveTransformNodeID(self.ReferenceToImageNode.GetID())        
        self.StylusTipToReferenceNode.SetAndObserveTransformNodeID(self.ReferenceToImageNode.GetID())
        
        
    def onSetRenderingCheckBox(self):
               
        if self.ui.renderingCheckBox.isChecked():
            volRenLogic = slicer.modules.volumerendering.logic()
            DisplayNode = volRenLogic.CreateDefaultVolumeRenderingNodes(self.volumeNode)
            DisplayNode.SetVisibility3D(True)
        else:
            volRenLogic = slicer.modules.volumerendering.logic()
            DisplayNode = volRenLogic.CreateDefaultVolumeRenderingNodes(self.volumeNode)
            DisplayNode.SetVisibility3D(False)
            
    def onSetTrackingCheckBox(self):
        if self.ui.trackingCheckBox.isChecked():           
            
            def tracking(caller, event):
                focalPoint = np.squeeze(slicer.util.arrayFromMarkupsControlPoints(self.focalMarkupsNode, world=True))
                slicer.vtkMRMLSliceNode.JumpAllSlices(slicer.mrmlScene, *focalPoint[0:3], slicer.vtkMRMLSliceNode.CenteredJumpSlice)
                
            self.observerView = self.TranCenterToRefernceNode.AddObserver(slicer.vtkMRMLTransformableNode.TransformModifiedEvent, tracking)
            
            self.red = slicer.util.getNode('vtkMRMLSliceNodeRed')
            self.red.SetSliceVisible(1)
            
            self.yellow = slicer.util.getNode('vtkMRMLSliceNodeYellow')
            self.yellow.SetSliceVisible(1)
            
            self.green = slicer.util.getNode('vtkMRMLSliceNodeGreen')
            self.green.SetSliceVisible(1)
                
        else:
            self.TranCenterToRefernceNode.RemoveObserver(self.observerView)
            
            self.red.SetSliceVisible(0)
            self.yellow.SetSliceVisible(0)
            self.green.SetSliceVisible(0)


    def onSetGetPointCloudButton(self):

        if self.ui.getPointsCheckBox.isChecked():
                       
            transformMatrix = vtk.vtkMatrix4x4()

            def getPoints(caller, event):

                self.StylusTipToReferenceNode.GetMatrixTransformToWorld(transformMatrix)

                matrix_arr = np.zeros((4, 4))
                for i in range(0, 4):
                    for j in range(0, 4):
                        matrix_arr[i, j] = (transformMatrix.GetElement(i, j))

                point = matrix_arr[:3,-1]
                self.surfFromMarkupsNode.AddControlPoint(point)
                time.sleep(0.1)
                
            self.observerGetPointCloud = self.StylusTipToReferenceNode.AddObserver(slicer.vtkMRMLTransformableNode.TransformModifiedEvent, getPoints)
        
        else:
            self.StylusTipToReferenceNode.RemoveObserver(self.observerGetPointCloud)
            
    
    def onSetSurfRegistrationButton(self):

        headModelNode = slicer.util.getNode("head")        
        self.ImageToModelNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", "ImageToModel")       

        self.logic = FiducialsToModelRegistrationLogic()       
        self.logic.run(self.surfFromMarkupsNode, headModelNode, self.ImageToModelNode)        
        
        slicer.app.processEvents()
        
        error = np.round(self.logic.ComputeMeanDistance(self.surfFromMarkupsNode, headModelNode, self.ImageToModelNode), 4)
        
        self.ui.surfStatusLabel.setText("Status: "+str(error)+" mm")
        self.ReferenceToImageNode.SetAndObserveTransformNodeID(self.ImageToModelNode.GetID())


class FiducialsToModelRegistrationLogic(ScriptedLoadableModuleLogic):


    def run(self, inputFiducials, inputModel, outputTransform, transformType=0, numIterations=100):
        """Run iterative closest point registration."""
        logging.info('Running iterative closest point registration')

        fiducialsPolyData = vtk.vtkPolyData()
        self.FiducialsToPolyData(inputFiducials, fiducialsPolyData)

        icpTransform = vtk.vtkIterativeClosestPointTransform()
        icpTransform.SetSource(fiducialsPolyData)
        icpTransform.SetTarget(inputModel.GetPolyData())
        icpTransform.GetLandmarkTransform().SetModeToRigidBody()
        if transformType == 1:
            icpTransform.GetLandmarkTransform().SetModeToSimilarity()
        if transformType == 2:
            icpTransform.GetLandmarkTransform().SetModeToAffine()
        icpTransform.SetMaximumNumberOfIterations(numIterations)
        icpTransform.Modified()
        icpTransform.Update()

        outputTransform.SetMatrixTransformToParent(icpTransform.GetMatrix())
        if slicer.app.majorVersion >= 5 or (slicer.app.majorVersion >= 4 and slicer.app.minorVersion >= 11):
            outputTransform.SetNodeReferenceID(slicer.vtkMRMLTransformNode.GetMovingNodeReferenceRole(),
                                                inputFiducials.GetID())
            outputTransform.SetNodeReferenceID(slicer.vtkMRMLTransformNode.GetFixedNodeReferenceRole(),
                                                inputModel.GetID())

        return True


    def ComputeMeanDistance(self, inputFiducials, inputModel, transform):
        surfacePoints = vtk.vtkPoints()
        cellId = vtk.mutable(0)
        subId = vtk.mutable(0)
        dist2 = vtk.mutable(0.0)
        
        locator = vtk.vtkCellLocator()
        locator.SetDataSet(inputModel.GetPolyData())
        locator.SetNumberOfCellsPerBucket(1)
        locator.BuildLocator()
        totalDistance = 0.0

        n = inputFiducials.GetNumberOfControlPoints()
        m = vtk.vtkMath()
        for fiducialIndex in range(0, n):
            
            originalPoint = [0, 0, 0]
            inputFiducials.GetNthControlPointPosition(fiducialIndex, originalPoint)
            transformedPoint = [0, 0, 0, 1]
            
            originalPoint.append(1)
            transform.GetTransformToParent().MultiplyPoint(originalPoint, transformedPoint)
            
            surfacePoint = [0, 0, 0]
            transformedPoint.pop()
            locator.FindClosestPoint(transformedPoint, surfacePoint, cellId, subId, dist2)
            totalDistance = totalDistance + math.sqrt(dist2)

        return (totalDistance / n)


    def FiducialsToPolyData(self, fiducials, polyData):

        points = vtk.vtkPoints()
        n = fiducials.GetNumberOfControlPoints()
        
        for fiducialIndex in range(0, n):
            p = [0, 0, 0]
            fiducials.GetNthControlPointPosition(fiducialIndex, p)
            points.InsertNextPoint(p)

        tempPolyData = vtk.vtkPolyData()
        tempPolyData.SetPoints(points)

        vertex = vtk.vtkVertexGlyphFilter()
        vertex.SetInputData(tempPolyData)
        vertex.Update()

        polyData.ShallowCopy(vertex.GetOutput())

