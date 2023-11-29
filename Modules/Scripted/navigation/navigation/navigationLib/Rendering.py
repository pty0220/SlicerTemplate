import numpy as np
import slicer
import qt
import vtk
import SimpleITK as sitk



l2n = lambda l: np.array(l)
n2l = lambda n: list(n)

def onNodeAdded(caller, event, calldata):
    node = calldata
    if isinstance(node, slicer.vtkMRMLVolumeNode):
        # Call showVolumeRendering using a timer instead of calling it directly
        # to allow the volume loading to fully complete.
        qt.QTimer.singleShot(0, lambda: showVolumeRendering(node))

def showVolumeRenderingCT(volumeNode):
    volRenLogic = slicer.modules.volumerendering.logic()
    displayNode = volRenLogic.CreateDefaultVolumeRenderingNodes(volumeNode)
    displayNode.SetVisibility(True)
    displayNode.GetVolumePropertyNode().Copy(volRenLogic.GetPresetByName('CT-Chest-Contrast-Enhanced'))


def showVolumeRendering(volumeNode):

    volRenLogic = slicer.modules.volumerendering.logic()
    displayNode = volRenLogic.CreateDefaultVolumeRenderingNodes(volumeNode)
    volumeName = volumeNode.GetName()

    propertyNode = displayNode.GetVolumePropertyNode()

    VolumeProperty = propertyNode.GetVolumeProperty()
    VolumeProperty.SetAmbient(1.0)
    VolumeProperty.SetDiffuse(0.0)
    VolumeProperty.SetSpecular(0.0)
    VolumeProperty.SetSpecularPower(1.0)

    array = slicer.util.arrayFromVolume(volumeNode)
    array_max = np.max(array)
    #array_max = 0.5

    if array_max>1.1:
        array_max= np.max(array)
    else:
        array_max= np.max(array)

    print("2Volume rendering Max:", array_max)
    opacityTransfer = vtk.vtkPiecewiseFunction()
    opacityTransfer.AddPoint(0, 0)
    opacityTransfer.AddPoint(array_max * 0.1, 0.)
    opacityTransfer.AddPoint(array_max * 0.3, 0.06)
    opacityTransfer.AddPoint(array_max * 0.5, 0.07)
    opacityTransfer.AddPoint(array_max * 0.6, 0.08)
    opacityTransfer.AddPoint(array_max * 0.7, 0.09)
    opacityTransfer.AddPoint(array_max * 0.85, 0.35)
    opacityTransfer.AddPoint(array_max * 0.99, 1)
    opacityTransfer.AddPoint(array_max * 1, 1.00)

    ctf = vtk.vtkColorTransferFunction()
    table = np.loadtxt("C:\\Users\\User\\AppData\\Local\\NA-MIC\\Slicer 5.0.3\\jet_table.txt")
    value_range = np.linspace(0.1*array_max, array_max, len(table))

    for i in range(len(table)):
        ctf.AddRGBPoint(value_range[i], table[i, 0], table[i, 1], table[i, 2])

    propertyNode.SetColor(ctf)
    propertyNode.SetScalarOpacity(opacityTransfer)
    
    # Add color node
    colorNode = slicer.mrmlScene.CreateNodeByClass("vtkMRMLProceduralColorNode")
    colorNode.UnRegister(None)  # to prevent memory leaks
    colorNode.SetName(slicer.mrmlScene.GenerateUniqueName("MyColormap"))
    colorNode.SetAttribute("Category", "MyModule")

    colorNode.SetHideFromEditors(False)
    slicer.mrmlScene.AddNode(colorNode)

    colorMap = colorNode.GetColorTransferFunction()
    colorMap.RemoveAllPoints()
    colorMap.DeepCopy(ctf)

    # Add an empty displayable node (you can only show a color legend if it belongs to a displayable node)
    displayableNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", volumeName + "_colorbar")
    displayableNode.CreateDefaultDisplayNodes()
    displayNode = displayableNode.GetDisplayNode()
    displayNode.AutoScalarRangeOff()
    displayNode.SetScalarRange(0.1 * array_max, array_max)

    # Display color legend
    displayNode.SetAndObserveColorNodeID(colorNode.GetID())
    colorLegendDisplayNode = slicer.modules.colors.logic().AddDefaultColorLegendDisplayNode(displayableNode)
    colorLegendDisplayNode.SetLabelFormat("%2.2f")
    colorLegendDisplayNode.SetTitleText(" ")
    colorLegendDisplayNode.SetNumberOfLabels(5)
    colorLegendDisplayNode.SetPosition(0.97, 0.5)
    colorLegendDisplayNode.SetSize(0.06, 0.4)

    TextProperty = colorLegendDisplayNode.GetLabelTextProperty()
    TextProperty.SetColor([0.0, 0.0, 0.0])
    TextProperty.SetFontFamilyToArial()



def saveITK(path, itkImage):
    writer = sitk.ImageFileWriter()
    writer.SetFileName(path)
    writer.Execute(itkImage)