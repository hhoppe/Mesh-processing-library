#!/usr/bin/perl -s
# Takes Stanford ply file (ascii or binary), and converts it to a mesh

use strict;

binmode(STDOUT);

@ARGV = ("-") if @ARGV==0;
die unless @ARGV==1;

open(FILE, "<@ARGV");
binmode(FILE);

my $binary = 0; my $little_endian = 0;
my $nvproperties = 0; my $nvpropbytes = 0;
my $ucharintensity = 0; my $ucharlist = 0; my $rgb_byte_offset = 0; my $nor_byte_offset = 0;
my $nv = 0; my $nf = 0; my $nstrips;

while (<FILE>) {
  s/\cM\cJ$/\cJ/;		# dos to unix
  s/ *$//;
  print "# $_";
  if (/^element vertex/) { ($nv) = /vertex\s+(\d+)$/ or die "? '$_'"; }
  if (/^element face/) { ($nf) = /face\s+(\d+)$/ or die "?"; }
  if (/^element tristrips/) { ($nstrips) = /tristrips\s+(\d+)$/ or die "?"; die "?" unless $nstrips==1; }
  if (/^property /) {
    if ($nf) {
      if (/^property uchar intensity/) {
	$ucharintensity = 1;
      } elsif (/^property list.*indices$/) {
      } else {
	die "?";
      }
      if (/list uchar int/) { $ucharlist = 1; }
    } elsif ($nstrips) {
      if (/^property list int int vertex_indices$/) {
      } else {
	die "?";
      }
    } else {
      if (/ vertex_indices$/) { next; }
      $nvproperties++;
      if (/ red$/) { $rgb_byte_offset = $nvpropbytes; }
      if (/ float nx$/) { $nor_byte_offset = $nvpropbytes; }
      if (/ uchar /) {
	$nvpropbytes += 1;
      } elsif (/ float /) {
	$nvpropbytes += 4;
      } else {
	die "?";
      }
    }
  }
  $little_endian = 1 if /^format.*binary_little_endian/;
  $binary = 1 if /binary_/;
  if (/^end_header$/) { last; }
}

warn "binary=$binary nvproperties=$nvproperties nvpropbytes=$nvpropbytes ucharlist=$ucharlist nv=$nv nf=$nf\n";

warn "processing vertices\n";
for (my $i = 1; $i<=$nv; $i++) {
  my $attrib = ""; my @F;
  if (!$binary) {
    $_ = <FILE>;
    @F = split(' ');
    @F==$nvproperties or die "?";
    if ($rgb_byte_offset && $nvproperties==6) {
      $attrib = sprintf(" {rgb=(%g %g %g)}", $F[3]/255., $F[4]/255., $F[5]/255.);
    }
  } else {
    read(FILE, $_, $nvpropbytes)==$nvpropbytes or die "?";
    @F = unpack("fff", pack("LLL", unpack($little_endian ? "VVV" : "NNN", substr($_, 0, 12))));
    die "?" unless @F==3;
    if ($rgb_byte_offset) {
      my $rgb = substr($_, $rgb_byte_offset, 3);
      # die "?" unless length($_)==3;
      $attrib = sprintf(" {rgb=(%g %g %g)}",
                        unpack("C", substr($rgb, 0, 1))/255.,
                        unpack("C", substr($rgb, 1, 1))/255.,
                        unpack("C", substr($rgb, 2, 1))/255.);
    }
    if ($nor_byte_offset) {
      my @N = unpack("fff", pack("LLL", unpack($little_endian ? "VVV" : "NNN", substr($_, $nor_byte_offset, 12))));
      $attrib = sprintf(" {normal=(%g %g %g)}", @N[0..2]);
    }
  }
  # print "Vertex $i  @F[0..2]$attrib\n";
  printf "Vertex $i  %g %g %g%s\n", @F[0..2], $attrib;
}

warn "processing faces\n";
if ($nstrips) {
  die "?" unless $binary;
  read(FILE, $_, 4)==4 or die "?";
  my $numi = unpack($little_endian ? "V" : "N", $_);
  my $ii = 0; my $nstrips = 0; my ($vi0, $vi1, $vi2);
  for (my $i = 0; $i<$numi; $i++) {
    read(FILE, $_, 4)==4 or die "?";
    my $vi = unpack($little_endian ? "V" : "N", $_);
    if ($vi==4294967295) { $nstrips++; $ii = 0; next; }
    $vi += 1; $ii++; $vi2 = $vi1; $vi1 = $vi0; $vi0 = $vi;
    next if $ii<3;
    $nf++;
    if ($ii&1) {
      print "Face $nf  $vi2 $vi1 $vi0\n";
    } else {
      print "Face $nf  $vi1 $vi2 $vi0\n";
    }
  }
  die "?" unless $ii==0;
  warn "nstrips=$nstrips nfaces=$nf\n";
} else {
  for (my $i = 1; $i<=$nf; $i++) {
    my @F;
    if (!$binary) {
      $_ = <FILE>;
      @F = split(' ');
    } else {
      if ($ucharintensity) { read(FILE, $_, 1); }
      if (!$ucharlist) {
	read(FILE, $_, 16)==16 or die "?";
	@F = unpack($little_endian ? "VVVV" : "NNNN", $_);
      } else {
	read(FILE, $_, 13)==13 or die "?";
	@F = unpack($little_endian ? "CVVV" : "CNNN", $_);
      }
    }
    die "?" unless @F==$F[0]+1;
    # if ($F[0]!=3) { warn "$F[0]\n"; }
    # die "?" unless $F[0]==3;
    shift @F;
    for my $a (@F) {
      $a += 1;
      if ($a<1 || $a>$nv) { warn "references to $a outside 1..$nv\n"; }
    }
    print "Face $i  @F\n";
  }
}
