
"""
Entry points for LR1110 Demo and Field tests

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

from .Job import (
    Executor,
    Logger,
    ResultLogger,
    ExecutorCriticalException,
)
from .FieldTestPost.Core import GeoLocServiceClientGnss, GeoLocServiceClientWifi
from .SerialExchange.DemoVcpReader import VcpReader
from .AppConfiguration import DemoAppConfiguration
from .Job import JobValidator
from json import dumps
from argparse import ArgumentParser, Action

from datetime import datetime
import pkg_resources
from os import path, makedirs, getcwd
from shutil import copyfile
from serial.serialutil import SerialException


from argparse import RawDescriptionHelpFormatter
from .Job import (
    UpdateAlmanacCheckFailure,
    UpdateAlmanacDownloadFailure,
    UpdateAlmanacWrongResponseException,
    UpdateAlmanacJob,
    LoggerException,
)

from .SerialExchange import (
    SerialHandler,
    CommunicationHandler,
    SerialHanlerEmbeddedNotSetException,
)

#from .main_almanac_update2 import entry_point_update_almanac



















class PrintJsonSchemaAction(Action):
    def __init__(self, *args, **kwargs):
        super().__init__(nargs=0, *args, **kwargs)

    def __call__(self, parser, values, namespace, option_string):
        print(dumps(JobValidator.get_schema(), indent=4))
        parser.exit()


def test_cli():
    version = pkg_resources.get_distribution("lr1110evk").version
    default_top_result_folder = getcwd()
    parser = ArgumentParser(description="Host companion for field tests of LR1110")
    parser.add_argument(
        "--top-results-folder",
        "-f",
        default=default_top_result_folder,
        help="Set the top directory where to store all the subfolders results. Default is current directory.",
    )
    parser.add_argument(
        "--result-folder-name",
        "-r",
        default=None,
        help="Name of the subdirectory that holds results files for the current run. Default is 'fieldTests_' with time appended.",
    )
    parser.add_argument(
        "--json-schema",
        "-j",
        default=False,
        action=PrintJsonSchemaAction,
        help="Print the json schema of the job file and return",
    )
    parser.add_argument("jobFile")
    parser.add_argument("--version", action="version", version=version)
    args = parser.parse_args()

    date_string = datetime.utcnow().strftime("%Y%m%d-%H%M%S")
    result_folder_name = args.result_folder_name or "fieldTests_{}".format(date_string)
    result_folder_path = path.join(args.top_results_folder, result_folder_name)

    result_filename = path.join(result_folder_path, "results.res")
    log_filename = path.join(result_folder_path, "logging.log")
    job_filename = args.jobFile
    job_filename_no_folder = path.basename(job_filename)

    try:
        makedirs(result_folder_path)
    except FileExistsError:
        print("The directory {} already exists. Aborting...".format(result_folder_path))
        exit(1)
    copyfile(job_filename, path.join(result_folder_path, job_filename_no_folder))

    result_logger = ResultLogger(result_filename)
    log_logger = Logger(log_filename)
    log_logger.print_also_on_stdin = True
    log_logger.log("Start")
    log_logger.log("Job filename: {}".format(job_filename))
    log_logger.log("Log filename: {}".format(log_filename))
    log_logger.log("Result filename: {}".format(result_filename))

    executor = Executor(result_logger, log_logger)
    executor.connect_serial()
    executor.load_jobs_from_file(job_filename)
    try:
        executor.execute()
    except ExecutorCriticalException as exec_crit_exc:
        log_logger.log("Terminating on critical exception: {}".format(exec_crit_exc))
    except KeyboardInterrupt:
        log_logger.log("User Keyboard Interrupt received")
        executor.reset()
    finally:
        executor.stop()
        log_logger.log("Terminating...")
        log_logger.terminate()
        result_logger.terminate()


def entry_point_demo():
#    print("Hello")
    print("GNSS Demo Start")
#    
#    entry_point_update_almanac()
#    
#    
#    return
    
    default_device = "/dev/ttyACM0"
#    default_baud = 921600
    default_baud = 115200
#    default_baud = 460800
    default_wifi_server_port_number = GeoLocServiceClientWifi.get_default_port_number()
    default_gnss_server_port_number = GeoLocServiceClientGnss.get_default_port_number()
    default_wifi_server_base = GeoLocServiceClientWifi.get_default_base_url()
    default_gnss_server_base = GeoLocServiceClientGnss.get_default_base_url()
    default_dry_run = False
    default_gnss_assisted_scan_coordinate = None
    default_reverse_geo_coding = False

    version = pkg_resources.get_distribution("lr1110evk").version
    parser = ArgumentParser()
    parser.add_argument(
        "approximateGnssServerLocalization",
        help="approximate coordinate sent to the server as initial localization for GNSS localization solver. Format is <latitude>,<longitude>,<altitude>",
    )
    parser.add_argument(
        "exactCoordinate",
        help="Exact coordinate. Sent to servers for error processing. Format is <latitude>,<longitude>,<altitude>",
    )
    parser.add_argument(
        "glsAuthenticationToken",
        help="HTTP header token to authenticate the Wi-Fi requests on LoRa Cloud Geolocation (GLS) server",
    )
#   parser.add_argument(
#       "token",
#       help="HTTP header token to authenticate the Wi-Fi requests on (DAS) server",
#   )
    parser.add_argument(
        "-s",
        "--wifi-server-base-url",
        help="Wifi server base address to use (default={}".format(
            default_wifi_server_base
        ),
        default=default_wifi_server_base,
    )
    parser.add_argument(
        "-p",
        "--wifi-server-port",
        type=int,
        help="Port number of the wifi server to contact (default={})".format(
            default_wifi_server_port_number
        ),
        default=default_wifi_server_port_number,
    )
    parser.add_argument(
        "-t",
        "--gnss-server-base-url",
        help="GNSS server base address to use (default={}".format(
            default_gnss_server_base
        ),
        default=default_gnss_server_base,
    )
    parser.add_argument(
        "-q",
        "--gnss-server-port",
        type=int,
        help="Port number of the GNSS server to contact (default={})".format(
            default_gnss_server_port_number
        ),
        default=default_gnss_server_port_number,
    )
    parser.add_argument(
        "-d",
        "--device-address",
        help="Address of the device connecting the lr1110 (default={})".format(
            default_device
        ),
        default=default_device,
    )
    parser.add_argument(
        "-b",
        "--device-baud",
        help="Baud for communication with the lr1110 (default={})".format(default_baud),
        default=default_baud,
    )
    parser.add_argument(
        "-n",
        "--dry-run",
        action="store_true",
        help="Do not contact geolocalization service (default={})".format(
            default_dry_run
        ),
        default=default_dry_run,
    )
    parser.add_argument(
        "-l",
        "--gnss-assisted-scan-approximate-localization",
        type=str,
        help="Set the estimated localization given to LR1110 for Assisted Start GNSS scan. Format is <latitude>,<longitude>,<altitude> (default is approximateGnssServerLocalization)",
        default=default_gnss_assisted_scan_coordinate,
    )
    parser.add_argument(
        "-r",
        "--reverse-geo-coding",
        action="store_true",
        help="Enable the reverse geo coding information in requests sent to server",
        default=default_reverse_geo_coding,
    )
    parser.add_argument(
        "-F",
        "--fake-date",
        action="store",
        help="Allows to fake the actual date to the one provided. Format is '{}'".format(
            DemoAppConfiguration.PARSE_DATE_FORMAT.replace("%", "%%")
        ),
        default=None,
    )
    parser.add_argument(
        "--verbose", "-v", help="Verbose", action="store_true", default=False
    )
    parser.add_argument("--version", action="version", version=version)
#    print(args)

    args = parser.parse_args()
    args.gls=False

    configuration = DemoAppConfiguration.from_arg_parser(args)


#    print(args)


#    return



    try:
        reader = VcpReader(configuration)
    except SerialException as serial_exception:
        print("Failed to initialize serial device: '{}'".format(serial_exception))
        exit(1)


    reader.start_read_vcp()

    try:
#        print("PASS4")
        reader.read_vcp_forever(args)
    except SerialException as serial_exception:
        print("Failure while reading serial: '{}'".format(serial_exception))
        exit(2)
    except KeyboardInterrupt:
        print("Bye\n")




def drive_field_tests():
    test_cli()






class ServerSolver:
    HUMAN_READABLE_NAME = None
    DEFAULT_DOMAIN = None
    DEFAULT_PATH = None
    HEADER_AUTH_TOKEN = None
    HEADER_CONTENT_TYPE_TOKEN = None

    def __init__(self, domain, path):
        self.__domain = domain
        self.__path = path

    def build_url(self):
        return "{}/{}".format(self.domain, self.path)

    def build_header(self, auth_value):
        return {
            self.get_header_auth_token(): auth_value,
            self.get_header_content_type_token(): "application/json",
        }

    @property
    def domain(self):
        return self.__domain

    @domain.setter
    def domain(self, domain):
        self.__domain = domain

    @property
    def path(self):
        return self.__path

    @path.setter
    def path(self, path):
        self.__path = path

    @classmethod
    def build_default_server_solver(cls):
        return cls(domain=cls.DEFAULT_DOMAIN, path=cls.DEFAULT_PATH,)

    @classmethod
    def get_header_content_type_token(cls):
        return cls.HEADER_CONTENT_TYPE_TOKEN

    @classmethod
    def get_header_auth_token(cls):
        return cls.HEADER_AUTH_TOKEN

    @classmethod
    def get_human_readable_name(cls):
        return cls.HUMAN_READABLE_NAME

    def __str__(self):
        return "{} ({})".format(self.get_human_readable_name(), self.build_url())


class GlsServerSolver(ServerSolver):
    HUMAN_READABLE_NAME = "GLS"
    DEFAULT_DOMAIN = "https://gls.loracloud.com"
    DEFAULT_PATH = "api/v3/almanac/full"
    HEADER_AUTH_TOKEN = "Ocp-Apim-Subscription-Key"
    HEADER_CONTENT_TYPE_TOKEN = "Content-Type"


class DasServerSolver(ServerSolver):
    HUMAN_READABLE_NAME = "DAS"
    DEFAULT_DOMAIN = "https://das.loracloud.com"
    DEFAULT_PATH = "api/v1/almanac/full"
    HEADER_AUTH_TOKEN = "Authorization"
    HEADER_CONTENT_TYPE_TOKEN = "Accept"


def entry_point_update_almanac():
    default_device = "/dev/ttyACM0"
    default_baud = 921600
    default_log_filename = "log.log"
    default_solver = DasServerSolver.build_default_server_solver()

    description = """EVK Demo App companion software that update almanac of LR1110 device.
    This software can fetch almanac from two servers:
       - GeoLocation Server (GLS): default URL is {}
       - Device and Application Server (DAS): default URL is {}
    By default, {} server is used.

    Optionnaly, almanac can also be read from file, which avoid contacting a server.""".format(
        GlsServerSolver.build_default_server_solver().build_url(),
        DasServerSolver.build_default_server_solver().build_url(),
        default_solver,
    )

    version = pkg_resources.get_distribution("lr1110evk").version
    parser = ArgumentParser(
        description=description, formatter_class=RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "token", help="Authentication token to the selected server",
    )
    parser.add_argument(
        "-g", "--gls", help="Use GLS server instead of DAS", action="store_true",
    )
    parser.add_argument(
        "-u",
        "--url-domain",
        help="Modify the domain name of the URL to use when fetching almanac",
        default=None,
    )
    parser.add_argument(
        "-f",
        "--almanac-file",
        help="Get the almanac information from a file instead of downloading it from web API. In this case the authentication server token is not used",
        default=None,
    )
    parser.add_argument(
        "-d",
        "--device-address",
        help="Address of the device connecting the lr1110 (default={})".format(
            default_device
        ),
        default=default_device,
    )
    parser.add_argument(
        "-b",
        "--device-baud",
        help="Baud for communication with the lr1110 (default={})".format(default_baud),
        default=default_baud,
    )
    parser.add_argument(
        "-l",
        "--log-filename",
        help="File to use to store the log (default={})".format(default_log_filename),
        default=default_log_filename,
    )
    parser.add_argument("--version", action="version", version=version)
    args = parser.parse_args()
#
#



    print(args)
    return

#
    # Check whether user wants GLS or DAS
    if args.gls is True:
        solver = GlsServerSolver.build_default_server_solver()
    else:
        solver = default_solver

    # Checks whether user wants to override URL domain or path
    if args.url_domain is not None:
        solver.domain = args.url_domain

    log_logger = Logger(args.log_filename)
    log_logger.print_also_on_stdin = True
#
#
    return
#






if __name__ == "__main__":
    print("TEST_CLI")
    test_cli()
