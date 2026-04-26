# net80211 Datapath - Transmit

## Overview

This document provides an overview for the transmit data path in
net80211, between the interface to the operating system, through net80211 and
into the driver.

The details about underlying implementations (eg how A-MPDU RX aggregation
is handled) will be covered in dedicated documents.

## Concurrency Notes

The transmit path(s), receive path and control / ioctl paths all run
in parallel and can be scheduled on multiple concurrently running
kernel threads.  It's important to keep this in mind.

## Transmit Path

There are two paths from the operating system layer into the net80211 transmit
path - the normal data path and the BPF / radiotap raw frame path.

It is important to note that both paths have no serialisation between
them, and multiple sending paths in the OS can and will queue frames
simultaneously across multiple concurrently executing threads/CPUs.
Please keep this in mind when reading the transmit handling and
how it interacts with 802.11 sequence numbering and encryption IV.

### Data Path - net80211

This is configured at the ifnet setup in ieee80211_vap_setup() -
the output path is ieee80211_vap_transmit().  This input path
takes 802.3 ethernet frames with no attached metadata (such as
rate control, transmit power, etc) - it is left up to the stack.

This hands the packet off to ieee80211_start_pkt() which will
perform the initial 802.11 destination lookup, query the node
state (eg whether it's in power save) and the VAP state (eg
is the vap itself in power state, or in a non-RUN state)
and drop or queue the frame appropriately.

It is then handed over to ieee80211_vap_pkt_send_dest() with
a destination ieee80211_node reference.

ieee80211_vap_pkt_send_dest() performs the bulk of the
net80211 transmit handling.  Packets will be queued here if the
destination node is in a power saving mode.

This includes:

 * Firstly - checking if the packet needs to be queued for
   power saving operation;
 * QoS classification via a call to ieee80211_classify();
 * BPF TX tap via a call to BPF_MTAP();
 * handling 802.11 encapsulation via ieee80211_encap() if required;
 * A-MPDU TX decisions, AMSDU and Atheros Fast-Frames decisions.

At this point the packet has been 802.11 encapsulated if required,
marked as needing encryption if required, and has been optionally
fragmented into a list of 802.11 fragments.

Finally, the packet / fragment packet chain is sent up to the driver via a call
to ieee80211_parent_xmitpkt().   The driver is expected to queue the
packet / fragment list or discard the packet / fragment list.  The specific
format of the mbuf chain and how ieee80211_node references are kept
is documented in ieee80211_parent_xmitpkt().

#### Notes on transmit path serialisation

Note that by default the IEEE80211_TX_LOCK() is held across the call to
ieee80211_encap() and ieee80211_parent_xmitpkt().  Drivers can register
that they properly handle 802.11 sequence number offloading via
IEEE80211_FEXT_SEQNO_OFFLOAD.  The lock is to ensure that packets
queued to the driver layer are added to the driver transmit queue
in the same order that they are 802.11 encapsulated - which sets the
802.11 sequence number.  Drivers which set IEEE80211_FEXT_SEQNO_OFFLOAD
indicate that they will assign the sequence number themselves - likely
at the same time that the transmit encryption IV number is assigned,
or simply offloaded in firmware - and thus this lock is not
required.

### Data path - Driver

The call ieee80211_parent_xmit() will call the driver ic->ic_transmit()
method.  At this point the driver can choose to queue / send the frame
(and take ownership of it), or return an error, and return it back
to net80211.  Currently net80211 will just free the mbuf and node reference
and return, but drivers should not assume that.

The mbuf passed in will be either a single 802.11/802.3 frame in an mbuf,
or a list of 802.11 fragments chained by m->m_nextpkt.  If the driver
has not set IEEE80211_FEXT_SEQNO_OFFLOAD then the packet will have
a sequence number assigned which the driver can fetch via M_SEQNO_GET().
The mbuf also holds an ieee80211_node reference.

(Note that fragments do not have sequence numbers assigned nor node
references.)

The driver needs to do a few things with this frame.  Notably if it's
an 802.3 offload device, it will be handed an 802.3 frame with no
802.11 information.  In that case, the driver just needs to queue
it for send to the hardware/firmware.

For devices which accept 802.11 frames, a few things are needed:

 * It needs to queue them for send, in the order they're given.
 * If there are any reasons the frames need to be buffered in the
   driver - eg node power state, asynchronous node/key/state updates -
   then they'll be buffered here until needed.
 * It needs to do any local hardware/firmware setup - rate control,
   transmit configuration, destination queue decisions, etc.
 * Hardware/firmware typically has some way to mark a frame as a type
   (control, data, management), whether RTS/CTS is needed, 
 * If IEEE80211_FEXT_SEQNO_OFFLOAD is set in the driver, it may need to
   allocate 802.11 sequence numbers via a call to ieee80211_output_seqno_assign().
 * If the frame is part of an MPDU (m->m_flags & M_AMPDU_MPDU) then
   the frame may need to be handled differently.  (For example rtwn(4)
   leaves sequence number assignment up to the firmware when A-MPDU is
   enabled.)
 * If the mbuf is marked as needing encryption (IEEE80211_FC1_PROTECTED
   is set in the 802.11 header) then the frame needs to be encrypted
   with the current encryption state via a call to ieee80211_crypto_encap().
 * Finally, the frame is queued to the hardware/firmware.

Again it is critical that the 802.11 sequence number and encryption be
called together in the same order.  This is typically done by the TX work
being done in a lock, or all frames being pushed into a single software
TX queue.

### Data path vs control path and the need to buffer frames

net80211 currently treats encryption key programming, VAP state
and other updates as synchronous calls.  For example, the
transmit path will call the driver to add a node, then
set the encryption keys and then queue a frame to be transmitted.

For devices which are programmed directly with no queued operations
(such as the ath(4) devices) the encryption key and node programming
is immediate.  However, for many other devices - firmware and
USB are two examples - these operations are asynchronous.
And these code paths tend to be in the transmit paths from
upper layers that may have locks held, so sleeping is not an option.

So for now this needs to be implemented in the driver itself.
It will need to maintain a per-node queue of transmit frames;
it will need to track asynchronous node creation/updates and
encryption key updates and buffer transmit frames for a node
until the node add/update and encryption key add/update is
completed.

### Transmit Completion Notification and Callbacks

TBD - ieee80211_tx_complete, the mbuf flags that handle the
completion notifications.

### Raw / BPF path

TODO - figure out (again) what if_output vs if_transmit are doing

* ieee80211_radiotap.c
* bpf_ieee80211_write()
* ifp->if_output() / ifp->if_input()

TODO: things to review / document

setup

* ieee80211_vap_create()

normal path

raw path

* ieee80211_raw_output()

non-data frame path

data frame path

power management / frame queueing

amsdu / fast frames queueing

initial encryption

driver hand-off

driver expectations - encryption, sequence number handling, etc
