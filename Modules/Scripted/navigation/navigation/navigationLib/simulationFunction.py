import sys
import os
import numpy as np
import math
import time
import SimpleITK as sitk


import navigationLib.helpFunction as hlp
# import skull_processing_vertical as sp_vertical
# import skull_processing as sp
# import make_transducer as sph

# from kwave_input_file import KWaveInputFile
# from kwave_output_file import KWaveOutputFile, DomainSamplingType, SensorSamplingType
# from kwave_data_filters import SpectralDataFilter
# from kwave_bin_driver import KWaveBinaryDriver
from navigationLib.niiCook import niiCook



l2n = lambda l: np.array(l)
n2l = lambda n: list(n)
start = time.time()

class makeSimulation():

    def __init__(self, path=False):

        print("Check init")
        ####################################################################
        # Material properties
        self.c_water = 1482  # [m/s]
        self.d_water = 1000  # [kg/m^3]
        self.a_water = 0.0253  # [Np/MHz/cm]

        self.c_bone = 3100  # [m/s]    # 2800 or 3100 m/s
        self.d_bone = 2200  # [kg/m^3]
        self.a_bone_min = 21.5  # [Np/MHz/m]
        self.a_bone_max = 208.9  # [Np/MHz/m]

        self.alpha_power = 2.0 # didn't consider dispersion

        ####################################################################
        # Source properties
        self.amplitude = 1  # source pressure [Pa]
        self.source_freq = 25e4  # frequency [Hz]
        self.ROC = 100  # [mm]     # transducer setting
        self.width = 85  # [mm]
        self.Tlength = 10
        self.focal_length = 92

        ####################################################################
        # Bounary condition
        self.boundary = 0
        self.PML_size = 10

        ####################################################################
        # Time step
        self.CFL = 0.1
        self.end_time = 100e-6
        self.points_per_wavelength = np.pi*2 # number of grid points per wavelength at f0

        ####################################################################
        # Recording
        self.recording = False

        ####################################################################
        # Back propagation
        self.PHASE = []
        self.AMP = []
        self.back_source = []
        self.optimizer_check = 0

        ####################################################################
        # Path
        if path == False:
            self.path = os.path.dirname(__file__)

        else:
            self.path = path
            
            try:
                os.mkdir(self.path)
            except:
                a=1

    def preprocessing(self, itk_image, target_pose):

        # f = open(os.path.join(self.path,'preprocessing.txt'), 'w')
        # sys.stdout = f

        s = time.time()

        target_pose = np.multiply(target_pose, (-1, -1, 1)).astype(float)

        ####################################################################
        ####################################################################
        # Source properties
        dx = self.c_water / (self.points_per_wavelength * self.source_freq)  # [m]
        dy = dx
        dz = dx

        ####################################################################
        # Grid_size contains the PML (default 20)
        grid_res = (dx, dy, dz)

        ####################################################################
        # Skull process

        ctCook = niiCook()
        if isinstance(itk_image, str):
            ctCook.readSavedFile(itk_image)
        else:
            ctCook.readITK(itk_image)

        # Remove air field
        skull_arr = ctCook.array
        skull_arr[skull_arr < 0] = 0
        skull_itk = ctCook.makeITK(skull_arr)

        # Find min & max threshold
        otsu_filter = sitk.OtsuThresholdImageFilter()
        otsu_filter.SetInsideValue(1)
        otsu_filter.SetOutsideValue(0)
        seg = otsu_filter.Execute(skull_itk)
        skull_min_threshold = 250
        skull_max_threshold = 1500

        skullCrop_itk, skullCrop_arr = ctCook.makeSimulationDomain(grid_res, self.focal_length, target_pose, self.width)

        self.domainCook = niiCook()
        self.domainCook.readITK(skullCrop_itk)

        # Segmentation
        skullCrop_arr[skullCrop_arr < skull_min_threshold] = 0
        skullCrop_arr[skullCrop_arr > skull_max_threshold] = skull_max_threshold

        skullCrop_itk = self.domainCook.makeITK(skullCrop_arr, self.path + "\\skullCrop_rotate_itk.nii")

        e = time.time()

        print("Perform skull processing")
        print("Simulation domain: ", skullCrop_arr.shape)
        print("Simulation dx: ", dx)
        print("Skull threshold:", skull_min_threshold)
        print("Skull max value:", skullCrop_arr.max())
        print("Resample time:",e-s)

        self.dx = dx
        self.grid_res = grid_res
        self.target_pose = target_pose
        self.skullCrop_arr = skullCrop_arr
        self.rawCrop_arr = skullCrop_arr.copy()
        self.skullCrop_itk = skullCrop_itk
        self.domain_shape = skullCrop_arr.shape
        self.target_idx = np.array(skullCrop_itk.TransformPhysicalPointToIndex(target_pose)).astype(int)
        self.p0 = np.zeros(self.skullCrop_arr.shape)

        # sys.stdout = sys.__stdout__
        # f.close()

    def make_transducer(self, tran_pose, normal = l2n([0,0,0])):

        ## Slicer to NIFTI coordinate
        tran_pose = np.multiply(tran_pose,  (-1, -1, 1)).astype(float)
        self.tran_pose = tran_pose
        self.tran_idx = np.array(self.skullCrop_itk.TransformPhysicalPointToIndex(tran_pose)).astype(int)

        if np.all(normal ==0):
            self.normal_idx = (self.target_idx - self.tran_idx)/np.linalg.norm(self.target_idx - self.tran_idx)
        else:
            normal = l2n(normal)*l2n([-1,-1,1])
            self.normal_idx = l2n([normal[2], normal[1], normal[0]])

        Tcenter = self.tran_idx
        Tnormal =  self.normal_idx

        Spos = hlp.make_transducer(self.ROC, self.width, self.dx, Tcenter, Tnormal)
        Spos = Spos.astype(int)

        if np.any(Spos[:,0] >= self.skullCrop_arr.shape[0])\
                or np.any(Spos[:,1] >= self.skullCrop_arr.shape[1])\
                or np.any(Spos[:,2] >= self.skullCrop_arr.shape[2]):
            self.Spos = -10
            self.p0 = np.ones(self.domain_shape)*(-10)
        else:
            p0 = self.skullCrop_arr.copy()
            p0[:,:,:] = 0
            p0[Spos[:,0],Spos[:,1],Spos[:,2]] = 1

            self.Spos = Spos
            self.p0 = p0
            self.trans_itk = self.domainCook.makeITK(self.p0*2000, self.path+"\\transducer.nii")

    def run_simulation(self):

        # f = open(os.path.join(self.path,'run_simulation.txt'), 'w')
        # sys.stdout = f

        start = time.time()
        print(" ")
        print(" ")
        print("################################")
        print("Start simulation")
        print("################################")
        print(" ")
        print(" ")
        print("####  Simulation specs  ####")
        print("Iso Voxel size: " + str(self.dx))

        print("CFL: " + str(self.CFL))
        print("end time: " + str(self.end_time))
        print("PPW: " + str(self.points_per_wavelength))

        ####################################################################
        # Source properties
        amplitude = self.amplitude       # source pressure [Pa]
        source_freq = self.source_freq     # frequency [Hz]

        ####################################################################
        # Material properties
        c_water = self.c_water      # [m/s]
        d_water = self.d_water      # [kg/m^3]
        a_water = self.a_water   # [Np/MHz/cm]

        c_bone = self.c_bone       # [m/s]    # 2800 or 3100 m/s
        d_bone = self.d_bone       # [kg/m^3]
        a_bone_min = self.a_bone_min   # [Np/MHz/cm]
        a_bone_max = self.a_bone_max  # [Np/MHz/cm]
        alpha_power = self.alpha_power

        ####################################################################
        # Grid properties
        dx = self.dx
        dy = dx
        dz = dx
        grid_res = self.grid_res

        ####################################################################
        # skull array
        skullCrop_arr = self.skullCrop_arr

        ####################################################################
        # Transducer
        p0 = self.p0


        ####################################################################
        # Time step
        CFL      = self.CFL
        end_time = self.end_time
        dt       = CFL * grid_res[0] / c_bone
        steps    = int(end_time / dt) + 1


        input_filename  ='kwave_in.h5'
        output_filename ='kwave_out.h5'

        ####################################################################
        # Skull process
        grid_size = skullCrop_arr.shape

        if np.all(skullCrop_arr==0):
            skull_max = 1
        else:
            skull_max = np.max(skullCrop_arr)

        print("Skull_max test", np.max(skullCrop_arr))
        ####################################################################
        # assign skull properties depend on HU value  - Ref. Numerical evaluation, Muler et al, 2017

        self.skullCrop_arr_normalized = (skullCrop_arr/skull_max)
        PI = 1 - self.skullCrop_arr_normalized

        ct_sound_speed = c_water*PI + c_bone*(1-PI)
        ct_density  = d_water*PI + d_bone*(1-PI)

        ct_att          = a_bone_min + (a_bone_max-a_bone_min)*np.power(PI, 0.5)
        ct_att[PI==1]   = a_water

        try:
            ct_sound_speed[self.lens==1] = self.lens_speed
            ct_density[self.lens == 1] = self.lens_density
            ct_att[self.lens == 1] = 163

        except:
            a=1

        ###################################################################
        # assign skull properties depend on HU value  - Ref. Multi resolution, Yoon et al, 2019
        # PI = skullCrop_arr/np.max(skullCrop_arr)
        # ct_sound_speed = c_water + (2800 - c_water)*PI
        # ct_density     = d_water + (d_bone - d_water)*PI
        # ct_att         = 0 + (20 - 0)*PI

        ####################################################################
        # Assign material properties
        sound_speed     = ct_sound_speed
        density         = ct_density
        alpha_np_MHz_m  = ct_att #[Np/MHz/m]

        alpha_np_m = alpha_np_MHz_m*pow(source_freq/1e6, alpha_power) #[Np/m]
        alpha_np_rad_m = alpha_np_m/pow(2*np.pi*source_freq, alpha_power) #[Np/(rad/s)/m]
        alpha_coeff = hlp.neper2db(alpha_np_rad_m, alpha_power) #[dB/MHz^y/cm]

        print("Max sound speed", np.max(ct_sound_speed))
        print("density", np.max(density))
        print("Max attenuation", np.max(alpha_coeff))
        print("alpha_power", self.alpha_power)

        # self.domainCook.makeITK(sound_speed, self.path+"\\sound_speed.nii")
        # self.domainCook.makeITK(density, self.path+"\\density.nii")
        # self.domainCook.makeITK(alpha_np_MHz_m, self.path+"\\alpha_coeff_np.nii")

        ####################################################################
        # Add PML SIZE fixed as 10
        PML_size = self.PML_size

        grid_size_PML = tuple(grid_size+l2n([PML_size*2, PML_size*2, PML_size*2]))
        sound_speed_PML = hlp.expand_matrix(sound_speed, PML_size)
        density_PML = hlp.expand_matrix(density, PML_size)
        alpha_coeff_PML = hlp.expand_matrix(alpha_coeff, PML_size)
        p0_PML = hlp.expand_matrix(p0, PML_size)

        ####################################################################
        # Define simulation input and output files
        print("## k-wave core input function")
        input_file  = KWaveInputFile(input_filename, grid_size_PML, steps, grid_res, dt, pml_size = PML_size)
        output_file = KWaveOutputFile(file_name=output_filename)


        ####################################################################
        # Transducer signal
        try:
            source_signal = self.source_signal
        except:
            source_signal = amplitude * np.sin((2*math.pi)*source_freq*np.arange(0.0, steps*dt, dt))


        ####################################################################
        # Open the simulation input file and fill it as usual
        with input_file as file:
            file.write_medium_sound_speed(sound_speed_PML)
            file.write_medium_density(density_PML)
            file.write_medium_absorbing(alpha_coeff_PML, alpha_power)
            file.write_source_input_p(file.domain_mask_to_index(p0_PML), source_signal, KWaveInputFile.SourceMode.ADDITIVE, c_water)

            sensor_mask = np.ones(grid_size_PML)
            file.write_sensor_mask_index(file.domain_mask_to_index(sensor_mask))

        # Create k-Wave solver driver, which will call C++/CUDA k-Wave binary.
        # It is usually necessary to specify path to the binary: "binary_path=..."
        driver = KWaveBinaryDriver()


        # Specify which data should be sampled during the simulation (final pressure in the domain and
        # RAW pressure at the sensor mask
        driver.store_pressure_everywhere([DomainSamplingType.MAX])
        if self.recording:
            driver.store_pressure_at_sensor([SensorSamplingType.RAW])


        # Execute the solver with specified input and output files
        driver.run(input_file, output_file)
        print("## Calculation time :", time.time() - start)


        #Open the output file and generate plots from the results
        with output_file as file:

            p_max = file.read_pressure_everywhere(DomainSamplingType.MAX)
            if self.recording:
                p_raw = file.read_pressure_at_sensor(SensorSamplingType.RAW)
                p_raw = np.squeeze(p_raw).transpose([1, 0])
                time_step = p_raw.shape[1]
                #p_raw = np.flip(p_raw)
                p_raw = p_raw.flatten()
                p_raw = np.reshape(p_raw, (self.domain_shape[0], self.domain_shape[1], self.domain_shape[2], time_step))
                p_raw = p_raw.transpose([2, 1, 0, 3])
                self.p_raw = p_raw

            # extract PML
            p_max = p_max[PML_size:-PML_size, PML_size:-PML_size, PML_size:-PML_size]

            try:
                p_max = p_max * (1 - self.lens)
            except:
                a=1

            result_itk = self.domainCook.makeITK(p_max, self.path+"\\forward.nii")

            self.p_max = p_max
            self.result_itk = result_itk

        # sys.stdout = sys.__stdout__
        # f.close()

        return result_itk


