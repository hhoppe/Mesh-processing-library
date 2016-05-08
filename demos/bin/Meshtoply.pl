#!/usr/bin/perl
# Takes a mesh and creates a text *.ply file.

# ply
# format ascii 1.0
# element vertex <number of verts>
# property float x
# property float y
# property float z
# property float nx
# property float ny
# property float nz
# element face <number of faces>
# property list uchar int vertex_indices
# end_header
# px py pz nx ny nz
# ...
# 3 v0 v1 v2
# ...

use strict;

binmode(STDOUT);

my $expectn = 0;
my @vertxyz = ();
my @faceids = ();
my $havenors = 0;

while (<>) {
  if (/^Vertex /) {
    my ($n, $x, $y, $z, $info) = /^Vertex (\d+)\s+(\S+)\s+(\S+)\s+(\S+)\s*(\{.*\})?\s*$/
      or die "? '$_'";
    die "?" unless @faceids==0;
    $expectn++; die "?n=$n expectn=$expectn\n" unless $n==$expectn;
    my $line = "$x $y $z";
    my $havenor = $info=~/normal=/;
    if (!@vertxyz) {
      $havenors = $havenor;
    } else {
      die "?" unless $havenor==$havenors;
    }
    if ($havenor) {
      $info =~ m@normal=\(\s*([^() ]+\s+[^() ]+\s+[^() ]+)\s*\)@ or die "? '$info'";
      $line .= " $1";
    }
    push(@vertxyz, $line);
  } elsif (/^Face /) {
    my ($verts, $info) = /^Face \d+\s+(\d[\d\s]*\d)\s*(\{.*\})?\s*$/ or die "? '$_'";
    my @verts = split(' ', $verts);
    my $nverts = @verts;
    for (@verts) { $_--; }
    $verts = join(' ', @verts);
    push(@faceids, "$nverts $verts");
  }
}

my $numv = @vertxyz;
my $numf = @faceids;
print "ply\n";
print "format ascii 1.0\n";
print "element vertex $numv\n";
print "property float x\n";
print "property float y\n";
print "property float z\n";
if ($havenors) {
  print "property float nx\n";
  print "property float ny\n";
  print "property float nz\n";
}
print "element face $numf\n";
print "property list uchar int vertex_indices\n";
print "end_header\n";
for (my $i = 0; $i<@vertxyz; $i++) {
  print $vertxyz[$i], "\n";
}
for (my $i = 0; $i<@faceids; $i++) {
  print $faceids[$i], "\n";
}
