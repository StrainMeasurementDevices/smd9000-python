"""
Command line application for the SMD9000 Package
"""
import time
import logging
import datetime
import click
from smd9000 import SMD9000, SMD9000Data, SMD9000DatastreamFormat


@click.group()
@click.option('--verbose', '-v', is_flag=True, help="Prints a detailed output of what's going on")
@click.option('--com-port', '-c', type=str, help="The port that the SMD9000 sensor is connected to")
@click.pass_context
def cli(ctx, verbose: bool, com_port):
    """
    Main SMD000 command line utility
    """
    ctx.ensure_object(dict)
    if verbose:
        logging.getLogger().setLevel(logging.DEBUG)
        logging.getLogger('cyflash-serial-transport').setLevel(logging.INFO)
    if com_port is None:
        print("Please supply a COM port")
        return
    ctx.obj['COM-PORT'] = com_port


@cli.command(no_args_is_help=True)
@click.option('--hardware-rev', '-h', is_flag=True, help="Gets the hardware revision")
@click.option('--firmware-rev', '-v', is_flag=True, help="Gets the firmware revision")
@click.pass_context
def info(ctx, hardware_rev, firmware_rev):
    """
    Gets information about the connected SMD000
    """
    c = SMD9000()
    if not c.connect(ctx.obj['COM-PORT']):
        print("Unable to connect to the SMD9000 sensor.")
        return

    rev = c.get_revisions()
    if hardware_rev:
        print("SMD9000 Hardware Revision: {:s}", rev.hardware_rev)
    if firmware_rev:
        print("SMD9000 Firmware Revision: {:s}", rev.firmware_rev)


@cli.command()
@click.option('--export-csv', '-o', default=None, help="Optional file to export the received data to. "
                                                       "Raises error if the file already exists (exclusive creation)")
@click.pass_context
def getdata(ctx, export_csv):
    """
    Starts a data stream from the sensor, printing and optionally recording the returned information
    """
    # pylint: disable=consider-using-with
    def datastream_def(d: SMD9000Data):
        user_str = f"{datetime.datetime.now().isoformat()}\t"
        for df in datastream_format:
            if df == SMD9000DatastreamFormat.SMD9000_DATA_FLOWRATE:
                user_str += f"{d.flowrate:f}\t"
            elif df == SMD9000DatastreamFormat.SMD9000_DATA_UPSTREAM_TOF:
                user_str += f"{d.up_tof:f}\t"
            elif df == SMD9000DatastreamFormat.SMD9000_DATA_DOWNSTREAM_TOF:
                user_str += f"{d.dn_tof:f}\t"
            elif df == SMD9000DatastreamFormat.SMD9000_DATA_SIG_STRENGTH:
                user_str += f"{d.sig:d}\t"
            elif df == SMD9000DatastreamFormat.SMD9000_DATA_STATUS_CODE:
                user_str += f"{d.stat:d}\t"
        user_str = user_str[:-1]
        print(user_str)
        if export_file is not None:
            user_str = user_str.replace('\t', ',')
            export_file.write(user_str+'\n')

    export_file = None
    c = SMD9000()
    if not c.connect(ctx.obj['COM-PORT']):
        print("Unable to connect to the SMD9000 sensor.")
        return
    c.set_stream_rate(10)
    datastream_format = c.get_datastream_format()
    if export_csv is not None:
        if not export_csv.endswith('.csv'):
            export_csv += '.csv'
        export_file = open(export_csv, 'x', encoding='utf-8')
    print("Press CTRL-C to exit")
    # Print and write the header
    header_txt = "Time\t"
    for df in datastream_format:
        if df == SMD9000DatastreamFormat.SMD9000_DATA_FLOWRATE:
            header_txt += "Flowrate\t"
        elif df == SMD9000DatastreamFormat.SMD9000_DATA_UPSTREAM_TOF:
            header_txt += "Upstream TOF\t"
        elif df == SMD9000DatastreamFormat.SMD9000_DATA_DOWNSTREAM_TOF:
            header_txt += "Downstream TOF\t"
        elif df == SMD9000DatastreamFormat.SMD9000_DATA_SIG_STRENGTH:
            header_txt += "Signal Strength\t"
        elif df == SMD9000DatastreamFormat.SMD9000_DATA_STATUS_CODE:
            header_txt += "Status Code\t"
    header_txt = header_txt[:-1]
    print(header_txt)
    if export_file is not None:
        export_file.write(header_txt.replace('\t', ',') + '\n')

    c.start_data_stream(datastream_def)
    try:
        while 1:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    c.stop_data_stream()
    if export_file is not None:
        export_file.close()
