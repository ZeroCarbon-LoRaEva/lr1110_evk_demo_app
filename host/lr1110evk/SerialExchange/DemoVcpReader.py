"""
Define serial VCP-like reader for demo class

 Revised BSD License
 Copyright Semtech Corporation 2020. All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Semtech corporation nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

from threading import Thread
import serial
#import time as t
import json
#import os.path
#import time
import re

from ..Job.GnssDateLocBuilder import GnssDateLocBuilder
from lr1110evk.BaseTypes import (
    ScannedGnss,
    ScannedMacAddress,
    ScannedMacAddressMalformedCsvLineException,
    ScannedGnssMalformedCsvLineException,
    Version,
    VersionException,
    ScannedGnssException,
)
from lr1110evk.FieldTestPost.Core import (
    RequestSender,
    ResponseNoCoordinateException,
    NoNavMessageException,
    SolverContactException,
)
from lr1110evk.FieldTestPost.Core.GeoLocServiceClientBase import (
    GeoLocServiceClientReverseGeoCoding,
    GeoLocServiceBadResponseStatus,
)
from datetime import datetime
import time
from lr1110evk.Job.KmlExport import kmlOutput

from ..main_almanac_update2 import entry_point_update_almanac

gTemp     = 0.0 # Temperature
gHumi     = 0.0 # Humidity
gLati     = 0.0 # Latitude
gLong     = 0.0 # Longitude
gEdgerssi = 0.0 # Edge RSSI
gGwrssi   = 0.0 # GW   RSSI
gWifi     = 0.0 # Wifi stations

class LocalizationResult:
    def __init__(self, coordinate, accuracy, geo_coding):
        self._coordinate = coordinate
        self._accuracy = accuracy
        self._geo_coding = geo_coding

    @staticmethod
    def from_response(response):
        coordinate = response.estimated_coordinates
        accuracy = response.loc_accuracy
        # There is no reverse geo coding information available... yet
        geo_coding = None
        return LocalizationResult(coordinate, accuracy, geo_coding)

    @staticmethod
    def from_response_and_geocoding(response, geo_coding):
        coordinate = response.estimated_coordinates
        accuracy = response.loc_accuracy
        return LocalizationResult(coordinate, accuracy, geo_coding)

    @property
    def coordinate(self):
        return self._coordinate

    @property
    def accuracy(self):
        return self._accuracy

    @property
    def geo_coding(self):
        return self._geo_coding

    def __str__(self):
        return "- Coordinate: {}\n- Reverse geo-coding: {}".format(
            self._coordinate, self._geo_coding
        )


class VcpReader:

#    SERIAL_TIMEOUT_S = 0.1
    SERIAL_TIMEOUT_S = 1
    permit_almanac_download = False
    
    def __init__(self, configuration):
        self.request_sender = RequestSender(configuration)
        self.__configuration = configuration
        self.__device = configuration.device_address
        self.__baud = configuration.device_baud
        self.geo_loc_service_gnss = configuration.gnss_server
        self.geo_loc_service_wifi = configuration.wifi_server
        self.approximated_coordinate_gnss_lr1110 = (
            configuration.approximate_gnss_lr1110_localization
        )
        self.approximated_coordinate_gnss_server = (
            configuration.approximate_gnss_server_localization
        )
        self.serial = serial.Serial(
            port=self.device, baudrate=self.baud, timeout=VcpReader.SERIAL_TIMEOUT_S
        )
        self.keep_reading_vcp = False
        self.thread = Thread(name="VcpReadThread", target=self.read_vcp_forever)
        self.__line_read_counter = 0
        self.gnss_data_builder = GnssDateLocBuilder()
        self.COMMAND_HANDLER = {
            "DATE": self.DateCommandHandler,
            "TEST_HOST": self.TestHostCommandHandler,
            "SEND": self.SendDataCommandHandler,
            "FLUSH": self.FlushDataCommandHandler,
            "RESULT": self.GetResultCommandHandler,
            "ALMA": self.almanac_updateHandler,
        }
        self.storage = list()
        self.localization_result = None
        self.result = None
        self.embedded_version = None
        self.permit_almanac_download = False

    def almanac_updateHandler(self, args):
        if self.permit_almanac_download == False:
            print("Don't Download Almanac Data")
            return
        print("Almanac Update")
# Close serial port for ascii data communication
#        self.serial.close()
        alm = entry_point_update_almanac
        alm.entry_point_update_almanac(args, self.serial)
# Open serial port for ascii data communication
#        self.serial = serial.Serial(
#            port=self.device, baudrate=self.baud, timeout=VcpReader.SERIAL_TIMEOUT_S
#        )
        time.sleep(0.01)


#   def save_file(self, response):
#       os.path.expanduser('~')
#       dt_now = datetime.now()
#
#       fjson = open(os.path.expanduser('~') + '/My Documents/Tracker_position.txt','w')
#       if ('null' in response.raw_response):
#           fjson.write("STATUS= FAIL\n")
#       else:
#           json_data = json.loads(response.raw_response)
#           print(json_data)
#           res_latitude = json_data['result']['latitude']
#           res_longitude = json_data['result']['longitude']
#           res_altitude = json_data['result']['altitude']
#           print ("Latitude=",res_latitude)
#           print ("Longitude=",res_longitude)
#           fjson.write("STATUS= SUCCESS\n")
#           fjson.write("Latitude= " + str(res_latitude) + "\n")
#           fjson.write("Longitude= " + str(res_longitude) + "\n")
#
#       fjson.write("Time= " + dt_now.isoformat() + "\n")
#       fjson.close()
#       return


    def FlushDataCommandHandler(self, args):
        self.storage.clear()

    def SendGnssDataToServer(self):
        request = self.request_sender.build_gnss_requests(self.storage)
        if self.embedded_version:
            request.embedded_versions = self.embedded_version
        self.print_if_verbose(
            "Request to send to server '{}': {}".format(
                self.geo_loc_service_gnss.server_address, request.to_json()
            )
        )
        if self.__configuration.dry_run:
            self.print_if_verbose("Dry run: not contacting server")
            return None
        response = self.request_sender.send_request_get_response(request)
        self.print_if_verbose(
            "Response from server: '{}'".format(response.raw_response)
        )
#       print("PASS1")
#       self.save_file(response)
        return response

    def SendWiFiDataToServer(self):
        request = self.request_sender.build_wifi_requests(self.storage)
        if self.embedded_version:
            request.embedded_versions = self.embedded_version
        self.print_if_verbose(
            "Request to send to server '{}': {}".format(
                self.geo_loc_service_wifi.server_address, request.to_json()
            )
        )
        if self.__configuration.dry_run:
            print("Dry run: not contacting server")
            return None
        response = self.request_sender.send_request_get_response(request)
        self.print_if_verbose(
            "Response from server: '{}'".format(response.raw_response)
        )
#       print("PASS2")
#       self.save_file(response)
        return response

    def ProduceResultWithReverseGeoLoc(self, geo_loc_response):
        reverse_geo_coding_client = GeoLocServiceClientReverseGeoCoding.from_default()
        reverse_geo_loc_response = reverse_geo_coding_client.call_service_and_get_response(
            geo_loc_response.estimated_coordinates
        ).reverse_geo_loc
        self.print_if_verbose("{}".format(reverse_geo_loc_response))
        result = LocalizationResult.from_response_and_geocoding(
            geo_loc_response, reverse_geo_loc_response
        )
        return result

    def ProduceResultWithoutReverseGeoLoc(self, geo_loc_response):
        result = LocalizationResult.from_response(geo_loc_response)
        return result

    def ProduceResultFromResponseAndConfiguration(self, geo_loc_response):
        if self.__configuration.request_reverse_geo_coding:
            result = self.ProduceResultWithReverseGeoLoc(geo_loc_response)
        else:
            result = self.ProduceResultWithoutReverseGeoLoc(geo_loc_response)
        return result

    def GetResultCommandHandler(self, args):
        global gLati
        global gLong

        self.print_if_verbose("Get result...")
        if self.result:
            geo_coding_ascii = "".join(
                [
                    letter if ord(letter) < 128 else "_"
                    for letter in str(self.result.geo_coding)
                ]
            )
            message = "{};{};{};{};{!s}".format(
                self.result.coordinate.latitude,
                self.result.coordinate.longitude,
                self.result.coordinate.altitude,
                self.result.accuracy,
                geo_coding_ascii,
            )
            gLati = self.result.coordinate.latitude
            gLong = self.result.coordinate.longitude
        else:
            message = "0;0;0;0;Error"
        print ("Result")
        print (message)
        self.print_if_verbose("Message: '{}'".format(message))
        self.serial.write(message.encode("ascii") + b"\x0D\x00")

    def SendDataCommandHandler(self, args):
        self.print_if_verbose("Send data to server...")
        start_time = time.perf_counter()
        # Erase self.result if it exists
        self.result = None
        try:
            response = self.SendGnssDataToServer()
        except NoNavMessageException:
            response = self.SendWiFiDataToServer()
            kml_scan_type = kmlOutput.SCAN_TYPE_WIFI
            data = [
                data for data in self.storage if isinstance(data, ScannedMacAddress)
            ]
            self.result = self.ProduceResultFromResponseAndConfiguration(response)
        except SolverContactException as solver_exception:
            print(
                "Exception when trying to contact solver: '{}'".format(solver_exception)
            )
            self.result = None
        else:
            kml_scan_type = kmlOutput.SCAN_TYPE_GNSS
            data = [data for data in self.storage if isinstance(data, ScannedGnss)]
            if not data:
                self.print_if_verbose("No GNSS data")
                return
            if len(data) > 1:
                self.print_if_verbose("Too many GNSS data")
                return
            data = data[0]
            self.result = self.ProduceResultFromResponseAndConfiguration(response)

        if self.result:
            print("Result from server:")
            print(self.result)
        else:
            print("No result available from server")

        if self.result:
            kml = kmlOutput("LR1110", "test.kml")
            date = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")

            kml.add_point(date, kml_scan_type, self.result.coordinate, data)

            if self.__configuration.actual_coordinate:
                kml.add_point(
                    date,
                    kmlOutput.SCAN_TYPE_REFERENCE_COORDINATES,
                    self.__configuration.actual_coordinate,
                    self.storage,
                )
            kml.save()
        end_time = time.perf_counter()
        self.print_if_verbose(
            "Elapsed time for sending data: {} seconds".format(end_time - start_time)
        )

    def DateCommandHandler(self, args):
        self.print_if_verbose("Date handler")
        latitude = self.approximated_coordinate_gnss_lr1110.latitude
        longitude = self.approximated_coordinate_gnss_lr1110.longitude
        altitude = self.approximated_coordinate_gnss_lr1110.altitude
        if self.__configuration.fake_date:
            gps_second = self.gnss_data_builder.date_to_gps_format(
                self.__configuration.fake_date
            )
        else:
            gps_second = self.gnss_data_builder.get_now_gps()

        self.print_if_verbose(
            "Date : {} s, Loc. tokens: {}, {}, {}".format(
                gps_second, latitude, longitude, altitude
            )
        )
        data_to_send = (
            str.encode(
                ",".join(
                    [
                        str(token)
                        for token in [gps_second, altitude, latitude, longitude,]
                    ]
                )
            )
            + b"\x0D\x00"
        )
        print("Reply2", data_to_send)
        self.serial.write(data_to_send)

    def TestHostCommandHandler(self, args):
        self.print_if_verbose("Test Host handler")
        print("Reply3", b"demooglog\x0D\x00")
        self.serial.write(b"demooglog\x0D\x00")

    def handle_command(self, command, args):
        print(command)
        self.print_if_verbose("Handle command '{}'...".format(command))
        try:
            command_handler = self.COMMAND_HANDLER[command]
        except KeyError:
            print("Unknown command '{}'".format(command))
            return
        try:
            command_handler(args)
        except ResponseNoCoordinateException as no_coordinate_except:
            status_from_server = no_coordinate_except.status_str
            self.result = None
            print("Error from solver: {}".format(status_from_server))
        except GeoLocServiceBadResponseStatus as bad_response_service:
            print("Error contacting solver: {}".format(bad_response_service))
            self.result = None
        except SolverContactException as solver_exception:
            print(
                "Exception when trying to contact solver: '{}'".format(solver_exception)
            )
            self.result = None

    def handle_comment(self, comment):
        print("Embedded: {}".format(comment))

    def handle_storing(self, blob):
        if blob.startswith(Version.VERSION_TOKEN):
            try:
                self.embedded_version = Version.from_string(blob)
                self.print_if_verbose(
                    "Embedded version: {}".format(self.embedded_version)
                )
                self.request_sender.device_eui = self.embedded_version.chip_uid
            except VersionException as version_exception:
                print("Failed to interpret version: '{}'".format(version_exception))
            return

        receive_time = datetime.utcnow()
        try:
            element_to_store = ScannedMacAddress.from_csv_line(receive_time, blob)
        except ScannedMacAddressMalformedCsvLineException:
            try:
                element_to_store = ScannedGnss.from_csv_line(receive_time, blob)
            except ScannedGnssMalformedCsvLineException:
                print("Cannot convert to GNSS or WiFi scan: '{}'".format(blob))
            except ScannedGnssException as scanned_exception:
                print(
                    "Cannot handle the expression '{}': {}".format(
                        blob, scanned_exception
                    )
                )
            else:
                self.storage.append(element_to_store)
                self.print_if_verbose("Stored GNSS '{}'".format(element_to_store))
        else:
            self.storage.append(element_to_store)
            self.print_if_verbose("Stored MAC '{}'".format(element_to_store))

    def handle_read_data(self, data, args):
        """ Main handler for data comming from VCP

        Any single line received from VCP goes through this handler.
        The actual implementation does nothing, and classes inheriting
        VCPInterpreterBase should redefine this method.

        Args:
            data (string): The line received from VCP

        """
        global gTemp     # Temperature
        global gHumi     # Humidity
        global gLati     # Latitude
        global gLong     # Longitude
        global gEdgerssi # Edge RSSI
        global gGwrssi   # GW   RSSI
        global gWifi     # Wifi stations

        dt_now = datetime.now()

        if data != "":
            self.print_if_verbose("\nReceived '{}'".format(data))
        if data.startswith("!"):
            command = data[1:].strip().upper()
            self.handle_command(command, args)

        elif data.startswith("#RSSI"):
            comment = data[1:].strip()
            self.handle_comment(comment)

            if data[9] == ",":
                gGwrssi = float(re.sub(r"\D", "", data[7:9]))
            else:
                gGwrssi = float(re.sub(r"\D", "", data[7:10]))

            if data[6] == "-":
                gGwrssi = gGwrssi * -1

        elif data.startswith("#Receive from TX $TH "):
            comment = data[1:].strip()
            self.handle_comment(comment)

            gTemp = float(re.sub(r"\D", "", data[34:38])) / 100
            if data[33] == "-":
                gTemp = gTemp * -1

            gHumi = float(re.sub(r"\D", "", data[49:53])) / 100

            if data[77] == " ":
                gEdgerssi = float(re.sub(r"\D", "", data[75:77]))
            else:
                gEdgerssi = float(re.sub(r"\D", "", data[75:78]))

            if data[74] == "-":
                gEdgerssi = gEdgerssi * -1

            print("DATE : "    ,dt_now   )
            print("gTemp    = ",gTemp    )
            print("gHumi    = ",gHumi    )
            print("gLati    = ",gLati    )
            print("gLong    = ",gLong    )
            print("gEdgerssi= ",gEdgerssi)
            print("gGwrssi  = ",gGwrssi  )
            print("gWifi    = ",gWifi    )

            #初期化
            gTemp     = 0.0 # Temperature
            gHumi     = 0.0 # Humidity
            gLati     = 0.0 # Latitude
            gLong     = 0.0 # Longitude
            gEdgerssi = 0.0 # Edge RSSI
            gGwrssi   = 0.0 # GW   RSSI
            gWifi     = 0.0 # Wifi stations

        elif data.startswith("#Received # of mac address "):
            gWifi = re.sub(r"\D", "", data[27:28])

        elif data.startswith("#"):
            comment = data[1:].strip()
            self.handle_comment(comment)
        elif data.startswith("@"):
            blob_to_store = data[1:].strip()
            self.handle_storing(blob_to_store)

        elif data.startswith("$"):
            if data != "":
                print("Unknown line: {}".format(data))

        else:
            if data != "":
                print("Unknown line: {}".format(data))

    def read_vcp_forever(self,args):
        """ Runtime VCP reader

        This method continuously read the VCP. Each line received
        triggers a call to VcpInterpreter.handle_read_data.
        The read is stopped when the variable self.keep_reading_vcp
        evaluate to False.

        """
        alm = entry_point_update_almanac
        while (1):

#           in_command = input("Command (1 .. Start without Almanac, 2 .. Download Almanac, 3 .. End Program) :") 

            in_command = "1"    # コマンド固定
            print('1 .. Start without Almanac') 

            if in_command == "1" or  in_command == "2":
                if in_command == "2":
                   self.permit_almanac_download = True
                else:
                   self.permit_almanac_download = False
#                print("Start GNSS", in_command)
                self.TestHostCommandHandler(args)
                while self.keep_reading_vcp:
                    try:
                        line = self.serial.readline().decode("ascii")
                    except UnicodeDecodeError as decode_exception:
                        line = None
                        print("Error on serial reading: '{}'".format(decode_exception))
                    if line:
                        self.__line_read_counter += 1
                        self.handle_read_data(line.strip(), args)
#                        print("Line=",line)
                    time.sleep(0.01)
                print(line)
            elif in_command == "2":
                print("Almanac Update")
                self.serial.close()
#
                alm.entry_point_update_almanac(args)
#
                self.serial = serial.Serial(
                    port=self.device, baudrate=self.baud, timeout=VcpReader.SERIAL_TIMEOUT_S
                )
                time.sleep(0.01)
#                exit(1)
#
            elif in_command == "3":
                print("Bye")
                exit(1)
            time.sleep(0.01)



    def read_vcp_forever_ORG(self):
        """ Runtime VCP reader

        This method continuously read the VCP. Each line received
        triggers a call to VcpInterpreter.handle_read_data.
        The read is stopped when the variable self.keep_reading_vcp
        evaluate to False.

        """
        while self.keep_reading_vcp:
            try:
                line = self.serial.readline().decode("ascii")
            except UnicodeDecodeError as decode_exception:
                line = None
                print("Error on serial reading: '{}'".format(decode_exception))
            if line:
                self.__line_read_counter += 1
                self.handle_read_data(line.strip())
                print("Line=",line)
#
#
            time.sleep(0.01)

    def start_read_vcp(self):
        """ Start the thread to read VCP

        This method sets the variable self.keep_reading_vcp to True and
        starts the thread.

        """
        self.keep_reading_vcp = True
        self.count_comment_lines = 0
        self.serial.flush()
        # self.thread.start()

    def stop_read_vcp(self):
        """ Stop the thread that read VCP

        This method sets the variable self.keep_reading_vcp to False and
        wait for the thread to join, therefore blocking the execution
        while waiting for the reading thread to terminate.

        """
        self.keep_reading_vcp = False
        self.thread.join()

    @property
    def device(self):
        """ Return the device address of the VCP

        Returns:
            string: Device address (eg. '/dev/ttyACM0')

        """
        return self.__device

    @property
    def baud(self):
        """ Return the device baud of the VCP

        Returns:
            int: Device baud (eg. 9600)

        """
        return self.__baud

    @property
    def line_read_counter(self):
        return self.__line_read_counter

    def print_if_verbose(self, message):
#        if self.__configuration.verbosity:
        print(message)
