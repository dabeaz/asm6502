	;; iodrv.asm
	;;
	;; Author : David Beazley (http://www.dabeaz.com)
	;; Copyright (C) 2010
	;; 
	;; Ohio Scientific superboard II I/O driver functions.  This code
	;; directly controls the ACIA chip to implement half-duplex
	;; serial communications.

	;; ACIA Ports
ACIA_TX     = 0xf001
ACIA_RX     = 0xf001
ACIA_STATUS = 0xf000           	

	;; IO Buffer

BUFSIZE     = 256
MSG_BUFFER  = 0x2000 - BUFSIZE
MSG_SIZE    = MSG_BUFFER + 1
MSG_DATA    = MSG_BUFFER + 2

	;; Send/Receive buffer definitions
	;; Note: for half-duplex operation, the send/recv buffers are the same
	
RECV_BUFFER = MSG_BUFFER
RECV_SIZE   = MSG_SIZE
RECV_DATA   = MSG_DATA
RECV_CMD    = MSG_BUFFER
	
SEND_BUFFER = MSG_BUFFER
SEND_SIZE   = MSG_SIZE
SEND_DATA   = MSG_DATA
SEND_CMD    = MSG_BUFFER
	
	;; Constants related to various communications primitions
FAST_ACK    = 0x03
BAD_ACK     = 0x00
	
	;; Command codes
COMMAND_RTS     = 0x0
COMMAND_PING    = 0x1
COMMAND_GETMEM  = 0x2
COMMAND_PUTMEM  = 0x3
COMMAND_GETVAR  = 0x4
COMMAND_PUTVAR  = 0x5
	
	;; Graphics Status locations
DISP_STATUS  = 0xd388
BAD_STATUS   = 0x42
GOOD_STATUS  = 0x47
BAD_COMMAND  = 0x43
DONE_STATUS  = 0x2e
	
INDIRECT     = 0xe8
MEM_LENGTH   = 0xe0
	
	;; Entry point of USR() requests.    At entry, the USR argument is extracted
	;; and packed into an initial request message that is repeatedly sent to
	;; a server until acknowledged.
	
0x1c00:
	;; Get the USR argument and send a message to listening clients
	JSR	0xae05		; This puts a 15-bit signed value in AE/AF

	;; Store the USR argument in an outgoing message.
	;; Outgoing message :  20 06 SEQ ARGL ARGH LRC
	
	INC	sequence	; Increment and store the request sequence number
	LDA	sequence
	STA	SEND_DATA
	LDA	0xae
	STA	SEND_DATA+2
	LDA	0xaf
	STA	SEND_DATA+1
	LDA	#0x20
	STA	SEND_BUFFER
	LDA	#0x06
	STA	SEND_SIZE

	;;  We now enter a loop waiting for the remote machine to acknowledge our message
	;;  The server must ack with sequence # 
	
initial_retry:	
	JSR	send_message
	LDX	#0xff
	LDY	#0xff
ack_poll:
	DEX
	CPX	#0x00
	BNE	_ack_poll1
	DEY
	CPY	#0x00
	BEQ	initial_retry

_ack_poll1:	
	LDA	#0x01
	BIT	ACIA_STATUS
	BEQ	ack_poll
	LDA	ACIA_RX
	CMP	#FAST_ACK
	BNE	initial_retry	; Response mismatch, keep trying
	JMP	do_commands
	
	;; ----------------------------------------------------------------------
	;; Receive a message.   This routine waits for and receives a message on the
	;; ACIA.  Messages are always encoded as follows
	;;
	;;    CMD SZ DATA LRC
	;;
	;;      CMD is of the form 0x2Y where Y is the command code
	;; 	SZ is the message size (in bytes)
	;; 	DATA is the command payload
	;;      LRC is a longitudinal redundancy check byte
	;;
	;; The routine does not return until a message has been successfully
	;; received.  Bad or corrupt messages are ignored.
	;; ----------------------------------------------------------------------

receive_message:
	LDX	#0		; Index into the recv buffer

recv_poll:
	LDA	#0x01
	BIT	ACIA_STATUS
	BEQ	recv_poll
	;;  Got a byte of data, read it and check for the start byte
	LDA	ACIA_RX
	STA	RECV_BUFFER, X
	;; Check the byte for proper form  (high nibble must be 2)
	AND	#0xf0
	CMP	#0x20
	BNE	recv_poll	;    Not found, go back to reading

	;; Got the start byte, get the message size
	LDA	#0x88
	STA	DISP_STATUS
	INX

recv_poll_sz:
	LDA	#0x01
	BIT	ACIA_STATUS
	BEQ	recv_poll_sz

	;; Got the size parameter
	LDA	ACIA_RX

	;; Store the size parameter
	STA	RECV_BUFFER,X
	INX
	INC	DISP_STATUS
	
	;; Read the rest of the message payload

recv_payload:
	LDA	#0x01
	BIT	ACIA_STATUS
	BEQ	recv_payload
	;;  Got a data byte, store it
	LDA	ACIA_RX
	STA	RECV_BUFFER,X

	;; Update display status
	INC	DISP_STATUS
	LDA	DISP_STATUS
	AND	#0x8f
	STA	DISP_STATUS
	
	INX
	CPX	RECV_SIZE
	BCC	recv_payload

	;; Compute the LRC of the message
	DEX
	LDA	RECV_BUFFER,X
	DEX

recv_lrc_calc:
	EOR	RECV_BUFFER,X
	CPX	#0x0
	BEQ	recv_lrc_done
	DEX
	JMP     recv_lrc_calc

	;; Done computing the LRC.  Accumulator should have a 0 in it

recv_lrc_done:

	CMP	#0x0
	BEQ	recv_done

	;; Bad message received
	LDA	#BAD_STATUS
	STA	DISP_STATUS
	JMP	receive_message	; try again

recv_done:
	LDA	#GOOD_STATUS
	STA	DISP_STATUS
	RTS

	;; ----------------------------------------------------------------------
	;; send_message
	;;
	;; Sends a stored message on the ACIA.
	;; ----------------------------------------------------------------------

send_message:
	LDX	#0x0		; Index in the send buffer
	LDY	SEND_SIZE       ; Index to LRC byte in send buffer
	DEY
	LDA	#0x0
	STA	SEND_BUFFER,Y	; Reset the LRC byte to 0
	
send_poll:
	;;  Poll the ACIA waiting for an ok to send
	LDA	#0x02
	BIT	ACIA_STATUS
	BEQ	send_poll

	;; Can send the next byte
	LDA	SEND_BUFFER,X
	STA	ACIA_TX
	;; Compute the LRC
	EOR	SEND_BUFFER,Y
	STA	SEND_BUFFER,Y

	INX
	CPX	SEND_SIZE
	BNE	send_poll

	;; Done sending
	RTS

	;; ----------------------------------------------------------------------
	;; do_command
	;;
	;; This routine processes the received command and calls an appropriate
	;; handler function.
	;; ----------------------------------------------------------------------

do_commands:
	JSR	receive_message
	
	LDX	#0x03		; Default response length
	LDY	#0x00
	;; Get the received command

	LDA	RECV_CMD
	STA	SEND_CMD
	AND	#0x0f		; Command code in lower nibble only

command_RTS:
	;; Command 0: Done. RTS.
	;; This returns from the USR subroutine
	CMP	#COMMAND_RTS
	BNE	command_PING
	STA	%0x5f
	LDA	#FAST_ACK	; Send a quick FAST_ACK in response.  Should be okay to send
	STA	ACIA_TX
	LDA	#DONE_STATUS
	STA	DISP_STATUS
	LDA	RECV_DATA+1
	LDY	RECV_DATA
	JSR	0xafc1
	RTS
	
command_PING:	
	;; Command 1 : PING
	CMP 	#COMMAND_PING
	BNE	command_GETMEM
	JMP	data_response

	;; Command 2 : GET
	;; Gets a region of memory.   
	;; Data format :  ADDRL ADDRH BYTES

command_GETMEM:
	CMP	#COMMAND_GETMEM
	BNE	command_PUTMEM

	LDA	RECV_DATA
	STA	%INDIRECT
	LDA	RECV_DATA + 1
	STA	%INDIRECT + 1
	LDA	RECV_DATA + 2
	STA	%MEM_LENGTH
	LDY	#0x00
	DEX
	
	;; Copy data into the send buffer.  Note: X contains the response length so far.
_cmd_getmem_copy:
	LDA	[INDIRECT, Y]
	STA	SEND_BUFFER, X
	INX
	INY
	CPY	%MEM_LENGTH
	BNE	_cmd_getmem_copy
	INX
	JMP	data_response

	;; Command 3 : PUTMEM
	;; Puts a region of memory
	;; Data format: ADDRL ADDRH DATA ...

command_PUTMEM:
	CMP 	#COMMAND_PUTMEM
	BNE	command_GETVAR

	LDA	RECV_DATA
	STA	INDIRECT
	LDA	RECV_DATA + 1
	STA	INDIRECT + 1
	LDX	#0x04
	LDY	#0x00
	DEC	RECV_BUFFER + 1
_cmd_putmem_copy:
	LDA	RECV_BUFFER, X
	STA	[INDIRECT, Y]
	INY
	INX
	CPX	RECV_BUFFER + 1
	BNE	_cmd_putmem_copy
	JMP	fast_response

	;; Command 4 : Get Variable.
	;; This returns the value of a BASIC variable.  For numbers, a 4-byte numeric
	;; value is returned.  For strings, the string data is returned.
	;; Side effects : Moves array storage up in memory to make room if needed.
	
command_GETVAR:
	CMP 	#COMMAND_GETVAR
	BNE	command_PUTVAR
	;; Get the variable name from the message and store it in 0x93/0x94
	LDA	RECV_DATA
	STA	%0x93
	LDA	RECV_DATA+1
	STA	%0x94
  	JSR	0xad53		; Look up variable  (name in 93/94?).  Address placed in 0x95/0x96
	      
	
	;; Copy variable value
	LDX	#0x02
	LDY	#0x00
	
	;; Check if string variable
	LDA	%0x94
	AND	#0x80
	BNE	getvar_copy_string

	;; Copy a numeric variable value
	
getvar_copy_num:	
	LDA	[0x95, Y]
	STA	SEND_BUFFER, X
	INX
	INY
	CPY	#0x04
	BNE	getvar_copy_num
	INX
	JMP	data_response

getvar_copy_string:
	;; Move the variable address to INDIRECT where we can use it
	LDA	[0x95, Y]	; Length
	STA	%MEM_LENGTH
	INY
	LDA	[0x95, Y]	; address (low)
	STA	%INDIRECT
	INY
	LDA	[0x95, Y]	; address (high)
	STA	%INDIRECT+1
	LDY	#0x00

	;; Copy the string data.  This must account for the possibility of a 0-length string
	;; not stored at any address.
getvar_copy_string_loop:
	CPY	%MEM_LENGTH
	BNE	getvar_copy_string_cont
	INX
	JMP	data_response

getvar_copy_string_cont:
	LDA	[INDIRECT, Y]
	STA	SEND_BUFFER, X
	INY
	INX
	JMP	getvar_copy_string_loop


	;; PUTVAR
	;;
	;; Stores a new variable value.   For numeric variables, this overwrites the 4 bytes of the numeric value.
	;; For strings.   New space may have to be allocated in the string storage space and the string data copied

command_PUTVAR:
	CMP	#COMMAND_PUTVAR
	BEQ	command_putvar_cont
	JMP	bad_command

command_putvar_cont:
	;; Get the variable name from the message and store it in 0x93/0x94
	LDA	RECV_DATA
	STA	%0x93
	LDA	RECV_DATA+1
	STA	%0x94
  	JSR	0xad53		; Look up variable  (name in 93/94?).  Address placed in 0x95/0x96

	;; Copy variable value
	LDX	#0x04
	LDY	#0x00
	
	;; Check if string variable
	LDA	%0x94
	AND	#0x80
	BNE	putvar_copy_string

	;; Store a numeric variable value
	
putvar_copy_num:
	LDA	RECV_BUFFER, X
	STA	[0x95, Y]
	INX
	INY
	CPY	#0x04
	BNE	putvar_copy_num
	STA	DISP_STATUS + 1
	JMP	fast_response
	
	;; Store a string variable value.
	;; To store the variable, the top of string memory (0x81/0x82) is decremented by the string length.
	;; The string data is then stored in the new space.   Finally, the variable information (length/addr) is updated
	
putvar_copy_string:

	LDA	RECV_SIZE
	SEC
	SBC	#0x05
	STA	%MEM_LENGTH
	
	;; First update the string storage variable
	LDA	%0x81		; String memory (low)
	SEC
	SBC	%MEM_LENGTH
	STA	%0x81
	LDA	%0x82
	SBC	#0x00
	STA	%0x82

	;; Copy the string data.  This must account for the possibility of a 0-length string
	;; not stored at any address.
putvar_copy_string_loop:
	CPY	%MEM_LENGTH
	BEQ	putvar_update_var

putvar_copy_string_cont:
	LDA	RECV_BUFFER, X
	STA	[0x81, Y]
	INY
	INX
	JMP	putvar_copy_string_loop

	;; Update the entry in the variable table
putvar_update_var:
	TYA
	LDY	#0x00
	STA	[0x95, Y]	; Store the variable length
	LDA	%0x81
	INY
	STA	[0x95, Y]	; Store variable address (low)
	LDA	%0x82
	INY
	STA	[0x95, Y]	; Store variable address (high)
	JMP	fast_response
	
bad_command:	
	LDA	#BAD_COMMAND
	STA	DISP_STATUS
	RTS

data_response:
	STX	SEND_SIZE
	JSR	send_message
	JMP 	do_commands

fast_response:
	;;  Poll the ACIA waiting for an ok to send.   When ready, send a single byte ACK 
	LDA	#0x02
	BIT	ACIA_STATUS
	BEQ	fast_response
	LDA	#FAST_ACK
	STA	ACIA_TX
	JMP	do_commands

	;;  Request sequence number
sequence:
	DATA	#0x00

	;; Message buffer
MSG_BUFFER:
	DATA	#0x21
	DATA	#0x06
	DATA	#'A'
	DATA	#'C'
	DATA	#'K'
	DATA	#0x00
	
