#!/usr/bin/perl
# Takes a mesh and creates an obj file
# Does not deal with any info on vertices, faces, or edges.

use strict;

binmode(STDOUT);

my $gid;

while (<>) {
  if (/^\#/) { print; next; }
  if (/^Vertex /) {
    my ($n,$x,$y,$z,$info)=/^Vertex (\d+)\s+(\S+)\s+(\S+)\s+(\S+)\s*(\{.*\})?\s*$/ or die "?";
    die "not renumbered?" unless $n=++$gid;
    printf("v %g %g %g\n",$x,$y,$z);
    # $mvv{$n}=$vnum++;
    next;
  }
  if (/^Face /) {
    my ($verts,$info)=/^Face \d+\s+(\d[\d\s]*\d)\s*(\{.*\})?\s*$/ or die "?";
    # warn "verts='$verts' info='$info'\n";
    my @verts=split(' ',$verts);
    print "f";
    for (@verts) { printf(" %d",$_); } # $mvv{$_}
    print "\n";
    next;
  }
}
