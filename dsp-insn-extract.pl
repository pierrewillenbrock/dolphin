#!/usr/bin/perl

use strict;

my $cl = "";

sub get_more() {
    my $l = <>;
    $cl .= $l;
    return $l;
}

my %nodes;

sub parse_node_opts( $ ) {
    my ($name) = @_;
    while(1) {
	if ($cl =~ s/^\s*([0-9a-zA-Z_]+)\s*=\s*\"((:?[^\"\\]|\\.)*)\"//s) {
	    $nodes{$name}->{opts}{$1}=$2;
	    next;
	}
	if ($cl =~ s/^\s*([0-9a-zA-Z_]+)\s*=\s*([0-9a-zA-Z_]+)\s+//s) {
	    $nodes{$name}->{opts}{$1}=$2;
	    next;
	}
	if ($cl =~ s/^\s*\]\s*;//s) {
	    last;
	}

	last unless get_more();
    }
}

sub parse_digraph( $ ) {
    my ($name) = @_;
    while(1) {
	if ($cl =~ s/^\s*([0-9a-zA-Z_]+)\s+->\s+([0-9a-zA-Z_]+)\s*;//s) {
	    push @{ $nodes{$1}->{"next"} },$2;
	    push @{ $nodes{$2}->{"prev"} },$1;
	    next;
	}
	if ($cl =~ s/^\s*([0-9a-zA-Z_]+)\s+\[//s) {
	    parse_node_opts($1);
	    next;
	}
	if ($cl =~ s/^\s*\}//s) {
	    last;
	}

	last unless get_more();
    }
}

while(1) {
    if ($cl =~ s/^\s*digraph\s+([0-9a-zA-Z_]+)\s+\{//s) {
	parse_digraph($1);
	next;
    }
    last unless get_more();
}

$cl =~ s/^\s*$//s || die;

my $n = (keys %nodes)[0];
while(exists $nodes{$n}->{"prev"}[0]) {
    $n = $nodes{$n}->{"prev"}[0];
}

while(defined $n) {
    if(exists $nodes{$n}->{opts}{label}) {
	print $nodes{$n}->{opts}{label}."\n"
    }
    last unless defined $nodes{$n}->{"next"};
    die if scalar(@ { $nodes{$n}->{"next"} } ) > 1;
    $n = $nodes{$n}->{"next"}[0];
}
