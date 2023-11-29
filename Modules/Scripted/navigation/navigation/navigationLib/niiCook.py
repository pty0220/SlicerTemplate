import sys
import os
import SimpleITK as sitk
import vtk
import numpy as np
import SimpleITK as sitk




l2n = lambda l: np.array(l)
n2l = lambda n: list(n)

from vtk.util import numpy_support as ns


class niiCook():

    def __init__(self):
        self.dummy = 0

    def readITK(self, image):

        spacing   = image.GetSpacing()
        origin    = image.GetOrigin()  ## bounds
        dimension = image.GetSize()
        direction = image.GetDirection()
        extent = (0, dimension[0] - 1, 0, dimension[1] - 1, 0, dimension[2] - 1)

        array = sitk.GetArrayFromImage(image)

        self.itkImage = image
        self.spacing = spacing
        self.origin = origin
        self.dimension = dimension
        self.extent = extent
        self.array = array
        self.direction = direction

    def readSavedFile(self, filePath):

        if filePath[-2:] == "gz":
            reader = sitk.ImageFileReader()
            reader.SetImageIO("NiftiImageIO")
            reader.SetFileName(filePath)

        elif filePath[-3:] == "nii":
            reader = sitk.ImageFileReader()
            reader.SetImageIO("NiftiImageIO")
            reader.SetFileName(filePath)

        elif filePath[-4:] == "nrrd":
            reader = sitk.ImageFileReader()
            reader.SetImageIO("NrrdImageIO")
            reader.SetFileName(filePath)

        else:
            reader = sitk.ImageSeriesReader()
            dicom_names = reader.GetGDCMSeriesFileNames(filePath)
            reader.SetFileNames(dicom_names)

        image     = reader.Execute()
        spacing   = image.GetSpacing()
        origin    = image.GetOrigin()  ## bounds
        dimension = image.GetSize()
        direction = image.GetDirection()
        extent = (0, dimension[0] - 1, 0, dimension[1] - 1, 0, dimension[2] - 1)

        array = sitk.GetArrayFromImage(image)

        self.direction = direction
        self.itkImage = image
        self.spacing = spacing
        self.origin = origin
        self.dimension = dimension
        self.extent = extent
        self.array = array

    def cropVolume(self, dimension, origin, spacing, original_image = 0):

        reference_image = sitk.Image(int(dimension[0]), int(dimension[1]), int(dimension[2]), sitk.sitkFloat32)
        reference_image.SetSpacing(spacing)
        reference_image.SetOrigin(origin)
        reference_image[:,:,:] = 0

        if original_image == 0:
            original_image = self.itkImage

        rigid_euler = sitk.Euler3DTransform()
        interpolator = sitk.sitkLinear
        default_value = -1000.0

        resampler = sitk.ResampleImageFilter()
        resampler.SetInterpolator(interpolator)
        resampler.SetReferenceImage(reference_image)
        resampler.SetOutputPixelType(sitk.sitkFloat32)
        resampler.SetDefaultPixelValue(default_value)
        resampler.SetTransform(rigid_euler)
        resampler.SetNumberOfThreads(15)
        crop_image = resampler.Execute(original_image)

        #crop_image = sitk.Resample(original_image, reference_image, rigid_euler, interpolator, default_value)

        return reference_image, crop_image

    def makeSimulationDomain(self, simul_spacing, focal_length, target_pos, make_even = True):

        simul_spacing = l2n(simul_spacing)*1000

        # for Optimal position 1.2

        max_point = target_pos + (focal_length * 1.2)
        min_point = target_pos - (focal_length * 1.2)

        bound = np.abs(max_point - min_point)
        domain = np.round(bound / simul_spacing)
        if make_even:
            domain = domain - domain % 10

        reference_image, crop_image = self.cropVolume(domain, min_point, simul_spacing)
        crop_array = sitk.GetArrayFromImage(crop_image)

        return crop_image, crop_array

    def makeITK(self, array, path=False):

        result_itk = sitk.GetImageFromArray(array)
        result_itk.SetSpacing(self.spacing)
        result_itk.SetOrigin(self.origin)
        result_itk.SetDirection(self.direction)
        #result_itk.SetDirection(self.direction)

        if path:
            writer = sitk.ImageFileWriter()
            writer.SetFileName(path)
            writer.Execute(result_itk)
        return result_itk
