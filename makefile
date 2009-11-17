#
# USRP tests makefile
#

LDFLAGS		= -lusrp -pthread -lliquid -lfftw3f -lfec
CPPFLAGS	:= -I . -I ./include -Wall -g -O2

local_src	:=		\
	basic.cc		\
	db_base.cc		\
	dbsrx.cc		\
	flex.cc			\
	lf.cc			\
	tvrx.cc			\
	usrp_io.cc		\
	usrp_rx_gain_correction.cc

local_progs	:=		\
	src/usrp_init_test.cc	\
	src/usrp_io_test.cc	\
	src/gr_usrp_rx_test.cc	\
	src/gr_usrp_tx_test.cc	\
	src/fmtx.cc 		\
	src/jammer.cc 		\
	src/tx_rrc.cc \
	src/test_usrp_standard_tx.cc \
	src/flexframe_tx.cc 	\
	src/flexframe_rx.cc	\
	src/packet_tx.cc \
	src/packet_rx.cc	\
	src/cr.cc		\
	src/tx_ofdmoqam.cc	\
	src/dsa_ofdmoqam.cc	\
	src/ofdmframe64_tx.cc	\
	src/ofdmframe64_rx.cc	\
	src/firpfbch_tx.cc	\
	src/usrp_rx_gain_correction_test.cc

sources		= $(addprefix lib/,$(local_src))
objects		= $(patsubst %.cc,%.cc.o,$(sources))
programs	= $(patsubst %.cc,%,$(local_progs))

all: $(programs)

$(objects): %.cc.o : %.cc
	g++ $(CPPFLAGS) -c $< -o $@

$(programs): % : %.cc $(objects)
	g++ $(CPPFLAGS) $(objects) $(LDFLAGS) $< -o $@

clean:
	$(RM) $(objects)
	$(RM) $(programs)
