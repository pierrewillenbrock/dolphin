#!/usr/bin/perl

my $f;
my $fn;

while(<>) {
    next unless $_ =~ /DSPJitI[LR]Nodes.cpp/;
    $_ =~ s/^.*E\[DSPLLE\]: //;
    if ($_ =~ /I[LR]Node dump/) {
	close $f;
	open $f,sprintf(">dsp-graph-%03d.dot",$fn);
	$fn++;
	next;
    }
    next if ($_ =~ /^!-+!\s$/);
    print $f $_;
}
