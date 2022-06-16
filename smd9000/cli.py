import os
import time
import logging
import typing
import click
import datetime
import smd9000


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
    c = smd9000.SMD9000()
    if not c.connect(ctx.obj['COM-PORT']):
        print("Unable to connect to the SMD9000 sensor.")
        return

    rev = c.get_revisions()
    if hardware_rev:
        print("SMD9000 Hardware Revision: {:s}".format(rev.hardware_rev))
    if firmware_rev:
        print("SMD9000 Firmware Revision: {:s}".format(rev.firmware_rev))


@cli.command()
@click.option('--export-csv', '-o', default=None, help="Optional file to export the received data to. "
                                                       "Raises error if the file already exists (exclusive creation)")
@click.pass_context
def getdata(ctx, export_csv):
    """
    Starts a data stream from the sensor, printing and optionally recording the returned information
    """
    def datastream_def(d: smd9000.SMD9000Data):
        user_str = '{:}\t{:f}\t{:f}\t{:f}\t{:d}\t{:d}'.format(datetime.datetime.now().isoformat(), d.flowrate, d.accum, d.tof, d.sig, d.stat)
        print(user_str)
        if export_file is not None:
            user_str = user_str.replace('\t', ',')
            export_file.write(user_str+'\n')

    export_file = None
    c = smd9000.SMD9000()
    if not c.connect(ctx.obj['COM-PORT']):
        print("Unable to connect to the SMD9000 sensor.")
        return
    c.set_stream_rate(10)
    if export_csv is not None:
        if not export_csv.endswith('.csv'):
            export_csv += '.csv'
        export_file = open(export_csv, 'x')
    print("Press CTRL-C to exit")
    print("Time\tFlow\tAccum Flow\tDTOF\tSignal Strength\tStatus")
    c.start_data_stream(datastream_def)
    try:
        while 1:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    c.stop_data_stream()
    if export_file is not None:
        export_file.close()


if __name__ == "__main__":
    cli()
