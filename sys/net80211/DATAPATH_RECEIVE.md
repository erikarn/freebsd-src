# net80211 Datapath - Receive

## Overview

This document provides an overview for receive data paths in
net80211, between the interface to the operating system, through net80211 and
into the driver.

The details about underlying implementations (eg how A-MPDU RX aggregation
is handled) will be covered in dedicated documents.

## Concurrency Notes

The transmit path(s), receive path and control / ioctl paths all run
in parallel and can be scheduled on multiple concurrently running
kernel threads.  It's important to keep this in mind.

## Receive Path

### Concurrency

There must only be one packet receive path into net80211.  The net80211 stack
does not currently handle multiple packet paths via locking, serialisation
or any other form of concurrency management.
