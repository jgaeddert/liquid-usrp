#
# USRP tests makefile
#

LDFLAGS	= -lusrp -lliquid

sources	:= src/usrp_init_test.cc src/gr_usrp_rx_test.cc

objects		= $(patsubst %.cc,%.cc.o,$(sources))
programs	= $(patsubst %.cc,%,$(sources))

all: $(programs)

$(objects): %.cc.o : %.cc
	g++ -Wall -g -O2 -c $< -o $@

$(programs): % : %.cc.o
	g++ -Wall -g -O2 $(LDFLAGS) $< -o $@

clean:
	$(RM) $(objects)
	$(RM) $(programs)
