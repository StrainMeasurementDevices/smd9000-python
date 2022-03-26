import os
import time
import logging
import typing
import click
import smd9000


@click.group()
@click.option('--verbose', '-v', is_flag=True, help="Prints a detailed output of what's going on")
@click.option('--com-port', '-c', default=None, type=str, help="Use a custom COM port instead of auto-finding it")
@click.pass_context
def cli(ctx, verbose: bool, com_port):
    """
    Main SMD000 command line utility
    """
    ctx.ensure_object(dict)
    if verbose:
        logging.getLogger().setLevel(logging.DEBUG)
        logging.getLogger('cyflash-serial-transport').setLevel(logging.INFO)

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
        print("SMD9000 Firmware Revision: {:d}.{:d}".format(rev.firmware_major_rev, rev.firmware_minor_rev))

if __name__ == "__main__":
    cli()
