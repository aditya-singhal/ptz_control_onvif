# Axis of the camera is according to the right hand thumb rule
# where thumb points towards +z axis.
# index finger points towards +y axis
# middle finger points towards +x axis
# If rotation of the second frame is in anticlockwise direction with respect to 
# the global frame(observed from origin), then the angle is positive, 
# otherwise the angle is negative.

# Pitch is rotation in the x-direction
# Roll is rotation in the y-direction
# Yaw is rotation in the z-direction
# To define the camera's rotation with respect to the global frame, it should be in the
# x, y and z sequence.

# Description:
# Optical zoom as per datasheet = 36x, but with zoom=1,it is going upto 33x
# After power reset camera goes to its last postion(i.e. the location before power reset), but 
# It doesnot do the appropriate zoom:
# Initial zoom was = 24, after power reset zoom = 16
# Initial zoom was = 12, after power reset zoom = 8

# Error:
# 1.There are some errors in onvif-zeep library for python3.
#   I tried AbsoluteMove function, but fields in the object returned from 'create_type function
#   are None.'

import sys
from time import sleep
from onvif import ONVIFCamera
import select
from math import cos, sin, radians, sqrt, acos, asin, pi
import numpy as np

#MACROS
DEFAULT_HOSTNAME_OF_CAMERA  = 'PTZCAMERA'

# Possible error codes
STATUS_OK = 0
STATUS_ERROR = 1
STATUS_CAMERA_NOT_CREATED = 2
STATUS_INVALID_USER = 3

# Camera default position,
CAMERA_DEFAULT_POSITION_PAN = 90
CAMERA_DEFAULT_POSITION_TILT = 90
CAMERA_DEFAULT_POSITION_ZOOM = 10
CAMERA_PAN_RANGE_MIN = 0
CAMERA_PAN_RANGE_MAX = 360

# Theoritically tilt is -16 to 90, but practically tilt is from -2.9 to 90 degrees
# When tilt value is +1, tilt angle is -2.9 degree, due to which there is a 3 degree
# error in the calculated and actual value
CAMERA_TILT_RANGE_MIN = -2.9
CAMERA_TILT_RANGE_MAX = 90
#optical zoom as per datasheet = 36x, but with zoom=1, it is going upto 33x
CAMERA_ZOOM_RANGE_MIN = 1
CAMERA_ZOOM_RANGE_MAX = 33

CAMERA_ZOOM_RANGE_PHYSICAL_IN_METER_MIN = 5
CAMERA_ZOOM_RANGE_PHYSICAL_IN_METER_MAX = 300

class camera(object):

    def __init__( self, ip, port, username, password, x, y, z, roll, pitch, yaw ):
        self.position_x = x
        self.position_y = y
        self.position_z = z
        self.yaw_radians = radians(yaw)
        self.pitch_radians = radians(pitch)
        self.roll_radians = radians(roll)

        # Initially this value is 1
        self.camera_creation_status = STATUS_OK

        try:
            self.camera_device = camera_onvif ( ip, port, username, password )
        except Exception as e:
            self.camera_creation_status = STATUS_CAMERA_NOT_CREATED
            print e
            return

        # Use these values instead of MACROS
        self.min_pan, self.max_pan = self.camera_device.onvif_get_pan_range()
        self.min_tilt, self.max_tilt = self.camera_device.onvif_get_tilt_range()
        self.min_zoom, self.max_zoom = self.camera_device.onvif_get_zoom_range()
        print ('Camera created')

    def __transform_object_location( self, global_x, global_y, global_z ):
        
        # This is the old logic, translation and rotation along any 1 axis is working fine
        #translation of the object
        #x_1 = global_x - self.position_x
        #y_1 = global_y - self.position_y
        #z_1 = global_z - self.position_z

        # #rotation of the object, y_1aw, zaxis
        # x_2 = x_1*(cos(self.yaw_radians)) - y_1*(sin(self.yaw_radians)) 
        # y_2 = x_1*(sin(self.yaw_radians)) + y_1*(cos(self.yaw_radians)) 
        # z_2 = z_1

        # #rotation of the object, pitch, x axis
        # x_3 = x_2
        # y_3 = y_2*(cos(self.pitch_radians)) - z_2*(sin(self.pitch_radians)) 
        # z_3 = y_2*(sin(self.pitch_radians)) + z_2*(cos(self.pitch_radians)) 

        # #rotation of the object, roll, y  axis
        # x_4 = x_3*(cos(self.roll_radians)) + z_3*(sin(self.roll_radians)) 
        # y_4 = y_3
        # z_4 = z_3*(cos(self.roll_radians)) - x_3*(sin(self.roll_radians)) 

        # print x_4, y_4, z_4

        p_1 = np.array([[global_x], [global_y], [global_z], [1]])
        #print('p_1',p_1)

        R_z = np.array([[cos(self.yaw_radians),-sin(self.yaw_radians),0], [sin(self.yaw_radians), cos(self.yaw_radians), 0],[0,0,1]])
        #print('R_z',R_z)
        R_x = np.array([[1,0,0], [0, cos(self.pitch_radians), -sin(self.pitch_radians)],[0,sin(self.pitch_radians),cos(self.pitch_radians)]])
        #print('R_x',R_x)
        R_y = np.array([[cos(self.roll_radians),0,sin(self.roll_radians)], [0, 1, 0],[-sin(self.roll_radians),0,cos(self.roll_radians)]])
        #print('R_y',R_y)
        R_1_2 = (R_z).dot(R_y).dot(R_x)
        #print('R_1_2',R_1_2)

        R_2_1 = np.transpose(R_1_2)
        #print('R_2_1',R_2_1)

        D_1_2 = np.array([[(self.position_x)],[(self.position_y)], [(self.position_z)]])
        #print('D_1_2',D_1_2)

        D_2_1  = -(R_2_1).dot(D_1_2)
        #print('D_2_1',D_2_1)

        T_2_1 =  np.array([[R_2_1[0][0],R_2_1[0][1],R_2_1[0][2],D_2_1[0][0]],[R_2_1[1][0],R_2_1[1][1],R_2_1[1][2],D_2_1[1][0]],[R_2_1[2][0],R_2_1[2][1],R_2_1[2][2],D_2_1[2][0]],[0,0,0,1]])
        # print('T_2_1',T_2_1)

        p_2 = (T_2_1).dot(p_1)

        return p_2[0], p_2[1], p_2[2]

    def __calculate_object_distance ( self, x, y, z ):
        object_distance = ((x*x)+(y*y)+(z*z))
        object_distance = sqrt( object_distance )
        return object_distance;

# Axis of the camera is according to the right hand thumb rule
# where thumb points towards +z axis.
# index finger points towards +y axis
# middle finger points towards +x axis
    def __calculate_pan_angle ( self, x, y ):
        if ( x==0 and y==0 ):
            pan_angle = 0
        else:
            object_distance_along_y_axis = ( y*y )
            object_distance_along_y_axis = sqrt(object_distance_along_y_axis)

            object_distance_in_x_y_plain = ( (x*x) + (y*y) )
            object_distance_in_x_y_plain = sqrt(object_distance_in_x_y_plain)

            pan_angle = object_distance_along_y_axis/object_distance_in_x_y_plain
            pan_angle = acos(pan_angle) * ( 180/pi )

            #2nd quadrant
            if( x>=0 and y<0 ):
                pan_angle = 180 - pan_angle

            #3rd quadrant
            elif( x<0 and y<0 ):
                pan_angle = 180 + pan_angle
            #4th quadrant
            elif( x<0 and y>=0 ):
                pan_angle = 360 - pan_angle;

        return 360 - pan_angle

# Also consider min/max values
    def __calculate_tilt_angle ( self, x, y, z ):
        if ( x==0 and y==0 and z==0 ):
            tilt_angle = 0
        else:        
            object_distance_along_z_axis = ( z*z )
            object_distance_along_z_axis = sqrt(object_distance_along_z_axis)

            object_distance = ( (x*x) + (y*y) + (z*z) )
            object_distance = sqrt(object_distance)
            
            tilt_angle = ( object_distance_along_z_axis ) / object_distance;
            tilt_angle = asin(tilt_angle) * 180/pi;

            if z < 0:
                tilt_angle = 90 - tilt_angle
            elif z >= 0:
                tilt_angle = 90 + tilt_angle
        
        return tilt_angle

    def __calculate_camera_values( self, x, y, z ):
        # find angle and distance of the camera from points
        pan_angle = self.__calculate_pan_angle( x, y )
        tilt_angle = self.__calculate_tilt_angle( x, y, z )
        object_distance = self.__calculate_object_distance( x, y, z )

        if tilt_angle > CAMERA_TILT_RANGE_MAX:
            tilt_angle = CAMERA_TILT_RANGE_MAX
        elif tilt_angle < 0:
            tilt_angle = 0

        slope_zoom = ( CAMERA_ZOOM_RANGE_PHYSICAL_IN_METER_MAX - CAMERA_ZOOM_RANGE_PHYSICAL_IN_METER_MIN ) / ( CAMERA_ZOOM_RANGE_MAX - CAMERA_ZOOM_RANGE_MIN)
        zoom = ( object_distance - CAMERA_ZOOM_RANGE_PHYSICAL_IN_METER_MIN + slope_zoom ) / slope_zoom

        return pan_angle, tilt_angle, zoom

    def set_home_position( self, x, y, z ):
        status = self.is_camera_created()
        if ( status != STATUS_OK ):
            return status

        self.home_position_x = x
        self.home_position_y = y
        self.home_position_z = z
        return STATUS_OK

    def go_to_home_position( self ):
        status = self.is_camera_created()
        if ( status != STATUS_OK ):
            return status
        
        status = self.move_camera( home_position_x, home_position_y, home_position_z )
        return status

    def get_position( self ):
        status = self.is_camera_created()
        if ( status != STATUS_OK ):
            return status

        # Map pan tilt and zoom to x y and z
        return self.camera_device.onvif_get_position()
    
    def move_camera( self, x, y, z ):
        status = self.is_camera_created()
        if ( status != STATUS_OK ):
            return status

        local_object_x, local_object_y, local_object_z = self.__transform_object_location( x, y, z )
        pan, tilt, zoom = self.__calculate_camera_values( local_object_x, local_object_y, local_object_z )
        zoom = round(zoom)
        # check if it lies in min and max ranges
        # print ( "pan:" + str(pan) )
        # print ( "tilt:" + str(tilt) )
        # print ( "zoom:"  + str(zoom) )
        return self.camera_device.onvif_move_camera( pan, tilt, zoom )

    def get_device_information( self ):
        status = self.is_camera_created()
        if ( status != STATUS_OK ):
            return status

        return self.camera_device.onvif_get_device_information()

    def set_hostname( self, hostname ):
        status = self.is_camera_created()
        if ( status != STATUS_OK ):
            return status

        return self.camera_device.onvif_set_hostname( hostname )

    def get_hostname( self ):
        status = self.is_camera_created()
        if ( status != STATUS_OK ):
            return status

        return self.camera_device.onvif_get_hostname()

    def get_system_date_and_time( self ):
        status = self.is_camera_created()
        if ( status != STATUS_OK ):
            return status

        return self.camera_device.onvif_get_system_date_and_time()

    def reboot_camera( self ):
        status = self.is_camera_created()
        if ( status != STATUS_OK ):
            return status

        return self.camera_device.onvif_reboot_camera()

    def is_camera_created( self ):
        return self.camera_creation_status

class camera_onvif(object):

    def __init__( self, ip, port, username, password ):

        try:
            self.my_camera = ONVIFCamera( ip, port, username, password )
        except Exception as e:
            print e
            raise Exception ( "Error in ONVIFCamera function" )

        # Device management service is available by default if camera is created
        # Create media service object
        self.media_service_object = self.my_camera.create_media_service()
        # Create ptz service object
        self.ptz_service_object = self.my_camera.create_ptz_service()
        # Create ptz service object
        self.imaging_service_object = self.my_camera.create_imaging_service()
        # Get target profile
        self.media_profile = self.media_service_object.GetProfiles()[0];
        # Get video sources
        video_sources = self.my_camera.media.GetVideoSources()
        # Save the range values
        nodes = self.my_camera.ptz.GetNodes()
        self.absolute_pan_value_min = nodes[0].SupportedPTZSpaces.AbsolutePanTiltPositionSpace[0].XRange.Min
        # print 'Absolute pan range Min:', self.absolute_pan_value_min
        self.absolute_pan_value_max = nodes[0].SupportedPTZSpaces.AbsolutePanTiltPositionSpace[0].XRange.Max
        # print 'Absolute pan range Max:',self.absolute_pan_value_max
        self.absolute_tilt_value_min = nodes[0].SupportedPTZSpaces.AbsolutePanTiltPositionSpace[0].YRange.Min
        # print 'Absolute tilt range Min:', self.absolute_tilt_value_min
        self.absolute_tilt_value_max = nodes[0].SupportedPTZSpaces.AbsolutePanTiltPositionSpace[0].YRange.Max
        # print 'Absolute tilt range Max:', self.absolute_tilt_value_max
        self.absolute_zoom_value_min = nodes[0].SupportedPTZSpaces.AbsoluteZoomPositionSpace[0].XRange.Min
        # print 'Absolute zoom range Min:', self.absolute_zoom_value_min
        self.absolute_zoom_value_max = nodes[0].SupportedPTZSpaces.AbsoluteZoomPositionSpace[0].XRange.Max
        # print 'Absolute zoom range Max:', self.absolute_zoom_value_max
        self.absolute_relative_pan_tilt_speed_min = nodes[0].SupportedPTZSpaces.PanTiltSpeedSpace[0].XRange.Min
        # print 'Absolute relative pan tilt speed range Min:', self.absolute_relative_pan_tilt_speed_min
        self.absolute_relative_pan_tilt_speed_max = nodes[0].SupportedPTZSpaces.PanTiltSpeedSpace[0].XRange.Max
        # print 'Absolute relative pan tilt speed range Max:', self.absolute_relative_pan_tilt_speed_max
        self.absolute_relative_zoom_speed_min = nodes[0].SupportedPTZSpaces.ZoomSpeedSpace[0].XRange.Min
        # print 'Absolute relative zoom speed range Min:', self.absolute_relative_zoom_speed_min
        self.absolute_relative_zoom_speed_max = nodes[0].SupportedPTZSpaces.ZoomSpeedSpace[0].XRange.Max
        # print 'Absolute relative zoom speed range Max:', self.absolute_relative_zoom_speed_max
        self.max_number_of_presets = nodes[0].MaximumNumberOfPresets
        # get presets
        self.presets = self.my_camera.ptz.create_type('GetPresets')
        self.presets.ProfileToken = self.media_profile._token
        self.presets = self.my_camera.ptz.GetPresets(self.presets)
        
        self.preset_count = 0
        self.preset_objects = []
        # print 'Max number of presets: ', self.max_number_of_presets

        self.slope_pan = ( CAMERA_PAN_RANGE_MAX - CAMERA_PAN_RANGE_MIN ) / ( self.absolute_pan_value_max - self.absolute_pan_value_min)
        self.slope_tilt = ( CAMERA_TILT_RANGE_MAX - CAMERA_TILT_RANGE_MIN ) / ( self.absolute_tilt_value_max - self.absolute_tilt_value_min)
        self.slope_zoom = ( CAMERA_ZOOM_RANGE_MAX - CAMERA_ZOOM_RANGE_MIN ) / ( self.absolute_zoom_value_max - self.absolute_zoom_value_min)
        # self.set_home_position( CAMERA_DEFAULT_POSITION_PAN, CAMERA_DEFAULT_POSITION_TILT, CAMERA_DEFAULT_POSITION_ZOOM)
        # set date time format ( check if needed ), timezone?

        param = self.my_camera.devicemgmt.create_type('SetHostname')
        param.Name = DEFAULT_HOSTNAME_OF_CAMERA
        self.my_camera.devicemgmt.SetHostname(param)
        print ('camera_onvif comamnds object created')

    def __get_users( self ):
        response = self.my_camera.devicemgmt.GetUsers()
        total_users = len(response)
        print ( 'Total number of users: ', total_users )
        return STATUS_OK, response

    def __create_user( self, username, password, userlevel ):
        try:
            param = self.my_camera.devicemgmt.create_type('CreateUsers')
            param.User = {'Username':username,'Password':password,'UserLevel':userlevel}
            response = self.my_camera.devicemgmt.CreateUsers(param)
            return STATUS_OK
        except Exception as e:
            print e
            print ( "Error in CreateUsers function" )
            return STATUS_ERROR

    def __remove_user( self, username ):
        try:
            param = self.my_camera.devicemgmt.create_type('DeleteUsers')
            param.Username = username
            response = self.my_camera.devicemgmt.DeleteUsers(param)
            print ( response )
            print ( username, "deleted" )
            return STATUS_OK
        except Exception as e:
            print e
            print ( "Error in DeleteUsers function" )
            return STATUS_ERROR

# This function return error when pan value is 1 and -1 and if tilt is -1
# "The home position is fixed and cannot be overwritten."
    def onvif_set_home_position( self, pan, tilt, zoom ):
        status = self.move_camera( pan, tilt, zoom )
        if status != STATUS_OK:
            print ( 'Error in move_camera' )
            return STATUS_ERROR
        else: 
            sleep(2)
            try:
                request = self.my_camera.ptz.create_type('SetHomePosition')
                request.ProfileToken = self.media_profile._token
                self.my_camera.ptz.SetHomePosition(request)
                print ( 'Home position set' )
                return STATUS_OK
            except Exception as e:
                print e
                print ( 'Error in SetHomePosition function' )
                return STATUS_ERROR

    def onvif_go_to_home_position( self ):
        try:
            request = self.my_camera.ptz.create_type('GotoHomePosition')
            request.ProfileToken = self.media_profile._token
            request.Speed.PanTilt._x = absolute_relative_pan_tilt_speed_max
            request.Speed.PanTilt._y = absolute_relative_pan_tilt_speed_max
            request.Speed.Zoom._x = absolute_relative_zoom_speed
            self.my_camera.ptz.GotoHomePosition(request)
            return STATUS_OK
        except Exception as e:
            print e
            print ( 'Error in GotoHomePosition function' )
            return STATUS_ERROR

    def onvif_get_pan_range( self ):
        return CAMERA_PAN_RANGE_MIN, CAMERA_PAN_RANGE_MAX

    def onvif_get_tilt_range( self ):
        return CAMERA_TILT_RANGE_MIN, CAMERA_TILT_RANGE_MAX
    
    def onvif_get_zoom_range( self ):
        return CAMERA_ZOOM_RANGE_MIN, CAMERA_ZOOM_RANGE_MAX

    def onvif_create_preset( self, pan, tilt, zoom ):
        
        if self.preset_count < self.max_number_of_presets:
            status = self.onvif_move_camera( pan, tilt, zoom )
            if status != STATUS_OK:
                print ( 'Error in move_camera' )
                return STATUS_ERROR
            else:
                sleep(2)
                try:
                    request = self.my_camera.ptz.create_type('SetPreset')
                    request.ProfileToken = self.media_profile._token
                    request.PresetName = str(self.preset_count)
                    request.PresetToken = self.presets[self.preset_count]._token
                    my_preset = self.my_camera.ptz.SetPreset(request)
                    self.preset_objects.append(my_preset)
                    self.preset_count += 1
                    print ( 'Preset created' )
                    return STATUS_OK, ( self.preset_count - 1 )
                except Exception as e:
                    print e
                    print ( 'Error in create preset function' )
                    return STATUS_ERROR, -1
        else:
            print ( 'Max number of presets reached' )
            return STATUS_ERROR, -1

    def onvif_go_to_preset( self, preset_id ):
        if preset_id < self.max_number_of_presets:
            try:
                preset = self.my_camera.ptz.create_type('GotoPreset')
                preset.ProfileToken = self.media_profile._token
                preset.PresetToken = self.preset_objects[preset_id]
                self.my_camera.ptz.GotoPreset(preset)
                return STATUS_OK
            except Exception as e:
                print e
                print ( 'Error in GotoPreset function' )
                return STATUS_ERROR                
        else:
            print ( 'Invalid preset id' )
            return STATUS_ERROR            

    def onvif_get_position( self ):
        request = self.my_camera.ptz.create_type('GetStatus')
        request.ProfileToken = self.media_profile._token
        response = self.my_camera.ptz.GetStatus(request)
        
        pan_value = response.Position.PanTilt._x
        # Tilt value returned by this function is multiplied by -1,
        tilt_value = response.Position.PanTilt._y * ( -1 )
        zoom_value = response.Position.Zoom._x
        #mapping physical values to camera values
        pan = ( pan_value * self.slope_pan ) - ( self.absolute_pan_value_min * self.slope_pan ) + CAMERA_PAN_RANGE_MIN
        tilt = ( tilt_value * self.slope_tilt ) - ( self.absolute_tilt_value_min * self.slope_tilt ) + CAMERA_TILT_RANGE_MIN
        zoom = ( zoom_value * self.slope_zoom ) - ( self.absolute_zoom_value_min * self.slope_zoom ) + CAMERA_ZOOM_RANGE_MIN
        return STATUS_OK, pan, tilt, zoom, response.MoveStatus.PanTilt, response.MoveStatus.Zoom

    def onvif_move_camera( self, pan, tilt, zoom ):
        # mapping physical values to camera values
        # zoom values are not mapping linearly with the input and output values
        # as conversion is not linear. 
        # Maybe same thing is happenning for pan and tilt but we are not able to identify that,
        # as we don't exactly by what angle camera is rotating.
        # print pan, tilt, zoom

        if zoom > CAMERA_ZOOM_RANGE_MAX:
            zoom = CAMERA_ZOOM_RANGE_MAX

        pan_value = ( ( pan - CAMERA_PAN_RANGE_MIN ) + ( self.absolute_pan_value_min * self.slope_pan ) ) / self.slope_pan
        tilt_value = ( ( tilt - CAMERA_TILT_RANGE_MIN ) + ( self.absolute_tilt_value_min * self.slope_tilt ) ) / self.slope_tilt

        # Hardware actual input values
        camera_actual_zoom_values = [0,0,0.062,0.093,0.121,0.152,0.184,0.215,0.243,0.274,0.305,0.333,0.364,0.395,0.427,0.454,0.486,0.517361111111,0.548,0.576,0.607,0.638,0.666,0.697,0.729,0.760,0.788,0.819,0.850,0.881,0.909,0.933,0.972,1]        
        zoom_value = camera_actual_zoom_values[int(zoom)]

        try:
            request = self.my_camera.ptz.create_type('AbsoluteMove')
            request.ProfileToken = self.media_profile._token
            request.Position.PanTilt._x = pan_value
            request.Position.PanTilt._y = tilt_value
            request.Position.Zoom._x = zoom_value
            request.Speed.PanTilt._x = self.absolute_relative_pan_tilt_speed_max
            request.Speed.PanTilt._y = self.absolute_relative_pan_tilt_speed_max
            request.Speed.Zoom._x = self.absolute_relative_zoom_speed_max
            self.my_camera.ptz.AbsoluteMove(request)
            print ( 'Camera moved' )
            return STATUS_OK
        except Exception as e:
            print e
            print ( 'Error in AbsoluteMove function' )
            return STATUS_ERROR

    def onvif_get_device_information( self ):
        response = self.my_camera.devicemgmt.GetDeviceInformation()
        return STATUS_OK, response.Manufacturer, response.Model, response.FirmwareVersion, \
            response.SerialNumber, response.HardwareId

    def onvif_set_hostname( self, hostname ):
        param = self.my_camera.devicemgmt.create_type('SetHostname')
        param.Name = hostname

        try:
            self.my_camera.devicemgmt.SetHostname(param)
            return STATUS_OK            
        except Exception as e:
            print e
            print ( 'Error in SetHostname function' )
            return STATUS_ERROR

    def onvif_get_hostname( self ):
        response = self.my_camera.devicemgmt.GetHostname()
        return STATUS_OK, response.Name

    def onvif_get_system_date_and_time( self ):
        response = self.my_camera.devicemgmt.GetSystemDateAndTime()
        return STATUS_OK, response.TimeZone.TZ, response.UTCDateTime, response.LocalDateTime

    def onvif_reboot_camera( self ):
        response = self.my_camera.devicemgmt.SystemReboot()
        return STATUS_OK, response

def camera_control_mock():

    camera_object = camera( '192.168.100.67', 80, 'admin', 'admin', 0, 0, -0.3, 0, 0, 0 )
    
    #camera_object.move_camera( -2, 15, 0 )
    #sleep(2)
    #camera_object.move_camera( 10, 10, 10 )
    #sleep(2)

if __name__ == '__main__':
    camera_control_mock()
    
