.. _rk3588_eth_sample:

RK3588 Ethernet Bring-Up
=========================

Overview
--------

This sample brings up the on-chip GMAC of the Rockchip RK3588 and requests
an IPv4 address via DHCP. Once the interface receives a lease, the sample
logs the assigned address and keeps the network interface active so that
the board can be reached with ICMP pings.

Requirements
------------

* Rockchip RK3588 board with an RGMII PHY (for example Orange Pi 5 Plus).
* A DHCP-enabled Ethernet network.

Building and Running
--------------------

.. code-block:: bash

   west build -b orange_pi5_plus samples/net/rk3588_eth
   west flash

On boot the console prints status messages similar to:

.. code-block:: text

   [00:00:01.234,000] <inf> rk3588_eth: waiting for IPv4 lease…
   [00:00:02.712,000] <inf> rk3588_eth: got IPv4 192.168.1.128

You can now verify connectivity using ``ping`` from another host on the
same network.
