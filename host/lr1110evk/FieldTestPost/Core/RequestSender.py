"""
Define Request sender class

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

from lr1110evk.BaseTypes import ScannedMacAddress, ScannedGnss
from .FileReader import FileReader
from .RequestBase import RequestWifiGls, RequestGnssGls
from .ResponseBase import ResponseBase
from .ExternalCoordinate import ExternalCoordinate
from .GeoLocServiceClientBase import GeoLocServiceBadResponseStatus
from .GeoLocServiceClientBase import GeoLocServiceTimeoutException


class RequestSenderException(Exception):
    pass


class MultipleScanTypesException(RequestSenderException):
    pass


class NoScanException(RequestSenderException):
    pass


class UnknownExternalId(RequestSenderException):
    def __init__(self, unknown_id):
        self.unknown_id = unknown_id

    def __str__(self):
        return "Unknown id '{}' while constructing request".format(self.unknown_id)


class TooManyNavMessagesException(RequestSenderException):
    def __init__(self, nav_message_list):
        self.nav_message_list = nav_message_list

    def __str__(self):
        return "Too many nav message reported for one job: {}".format(
            ",".join(self.nav_message_list)
        )


class NoNavMessageException(RequestSenderException):
    def __str__(self):
        return "No NAV message available"


class SolverContactException(RequestSenderException):
    def __init__(self, reason):
        self.reason = reason

    def __str__(self):
        return "Exception when contacting solver. Reason: '{}'".format(self.reason)


class RequestSender:
    def __init__(self, configuration):
        self.configuration = configuration
        self.TYPE_REQUEST_MAPPER = {
            ScannedGnss: self.build_gnss_requests,
            ScannedMacAddress: self.build_wifi_requests,
        }
        self.GEOLOC_SERVICE_MAPPER = {
            RequestWifiGls: self.configuration.wifi_server,
            RequestGnssGls: self.configuration.gnss_server,
        }
        self.device_eui = None

    def build_wifi_requests(self, mac_addresses):
        wifi_data = [
            data for data in mac_addresses if isinstance(data, ScannedMacAddress)
        ]
        request = RequestWifiGls()
        request.macs = wifi_data
        return request

    def build_gnss_requests(self, gnss_scan):
        gnss_data = [data for data in gnss_scan if isinstance(data, ScannedGnss)]
        if len(gnss_data) > 1:
            raise TooManyNavMessagesException(gnss_data)
        if not gnss_data:
            raise NoNavMessageException()
        gnss_data = gnss_data[0]
        utc_time = gnss_data.instant_scan
        request = RequestGnssGls(
            payload=gnss_data.nav_message,
            timestamp=utc_time,
            aiding_coordinate=self.configuration.approximate_gnss_server_localization,
        )
        return request

    def get_geo_loc_service_for_request(self, request):
        return self.GEOLOC_SERVICE_MAPPER[request.__class__]

    def send_request_get_response(self, request):
        geoloc_service = self.get_geo_loc_service_for_request(request)
        try:
            response = geoloc_service.call_service_and_get_response(request.to_json())
            return response
        except GeoLocServiceBadResponseStatus as bad_response:
            raise SolverContactException(
                reason="{}: {}".format(
                    bad_response.bad_http_code_text, bad_response.erroneous_response
                )
            )
        except GeoLocServiceTimeoutException as excp_timeout:
            print("GeoLocServiceTimeoutException Timeout")

    def send_request(self, request):
        response = self.send_request_get_response(request)
        self.print_if_verbose(
            "Raw response from server:\n - HTTP: {}\n - {}\n".format(
                response.http_code, response.raw_response
            )
        )
        coordinate = response.estimated_coordinates
        accuracy = response.loc_accuracy
        self.print_if_verbose(
            "Coordinates: {}\nAccuracy: {}".format(coordinate, accuracy)
        )
        return coordinate

    def build_request_group_iterator_from_result_lines(self, result_lines):
        key_scan_result_groups = FileReader.generate_result_groups(result_lines)
        for key_scan_group in key_scan_result_groups:
            # key = key_scan_group[0]
            group_result_lines = list(key_scan_group[1])
            # date = group_result_lines[0].date
            scan_info = [grp.scan_info for grp in group_result_lines]

            scan_info_types = {scan.__class__ for scan in scan_info}
            if ScannedMacAddress in scan_info_types:
                if ScannedGnss in scan_info_types:
                    raise MultipleScanTypesException()
                else:
                    scan_info_type = ScannedMacAddress
            else:
                if ScannedGnss in scan_info_types:
                    scan_info_type = ScannedGnss
                else:
                    yield group_result_lines, None
                    continue

            if self.is_wifi_deactivated():
                if scan_info_type is ScannedMacAddress:
                    continue
            request = self.TYPE_REQUEST_MAPPER[scan_info_type](scan_info)
            yield group_result_lines, request

    def is_wifi_deactivated(self):
        return self.configuration.deactivate_wifi_requests

    def print_if_verbose(self, output):
        if self.configuration.verbosity:
            print(output)
