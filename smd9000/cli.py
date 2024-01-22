"""
Command line application for the SMD9000 Package
"""
import time
import logging
import datetime
import typing

import click
from smd9000 import SMD9000, DataSet, StreamFormat


def pretty_print_datastream(df: StreamFormat) -> str:
    """
    Returns a pretty print text for a given datastream format
    """
    if df == StreamFormat.FLOWRATE:
        return "Flowrate"
    if df == StreamFormat.AMPLITUDE:
        return "Amplitude"
    if df == StreamFormat.STATUS_WORD:
        return "Status Word"
    # Nothing returned, must be error in the given df variable
    raise UserWarning("Invalid Datastream Format")


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
    if com_port is None:
        print("Please supply a COM port")
        return
    ctx.obj['COM-PORT'] = com_port


@cli.command()
@click.option('--datastream-format', '-d', is_flag=True, help="Gets the current datastream format")
@click.pass_context
def info(ctx, datastream_format):
    """
    Gets information about the connected SMD000
    """
    c = SMD9000()
    if not c.connect(ctx.obj['COM-PORT']):
        print("Unable to connect to the SMD9000 sensor.")
        return

    rev = c.read_info()
    print(rev)

    if datastream_format:
        d_format = c.get_stream_format()
        txt = ""
        for df in d_format:
            txt += pretty_print_datastream(df) + ', '
        print("Datastream Format:", txt[:-2])


@cli.command()
@click.option('--export-csv', '-o', default=None, help="Optional file to export the received data to. "
                                                       "Raises error if the file already exists (exclusive creation)")
@click.pass_context
def getdata(ctx, export_csv):
    """
    Starts a data stream from the sensor, printing and optionally recording the returned information
    """
    # pylint: disable=consider-using-with
    def datastream_def(dd: typing.List[DataSet]):
        for d in dd:
            user_str = f"{datetime.datetime.now().isoformat()}\t"
            for df in datastream_format:
                if df == StreamFormat.FLOWRATE:
                    user_str += f"{d.flow:f}\t"
                elif df == StreamFormat.AMPLITUDE:
                    user_str += f"{d.sig:d}\t"
                elif df == StreamFormat.STATUS_WORD:
                    user_str += f"{d.status.str:s}\t"
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
    datastream_format = c.get_stream_format()
    if export_csv is not None:
        if not export_csv.endswith('.csv'):
            export_csv += '.csv'
        export_file = open(export_csv, 'x', encoding='utf-8')
    print("Press CTRL-C to exit")
    # Print and write the header
    header_txt = "Time\t"
    for df in datastream_format:
        header_txt += pretty_print_datastream(df) + '\t'
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
