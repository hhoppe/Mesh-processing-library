#!/usr/bin/perl

package Materials_lib;
use 5.006;
require Exporter;
use Carp;

@ISA = qw(Exporter);
# @EXPORT = qw(mat_to_rgb);
@EXPORT_OK = qw(mat_to_rgb);

use strict;

my %materials_rgb;
my %warning;

for my $dir (".", @INC) {
  my $file = "$dir/Materials_rgb.db";
  open(MAT, $file) or next;
  while (<MAT>) {
    next if (/^\s*$/ || /^\#/);
    croak "? $_" unless my ($mat, $rgb_string) = /^"(.*)"\s+"(.*)"$/;
    croak "already defined $mat" unless !defined($materials_rgb{$mat});
    $materials_rgb{$mat} = $rgb_string;
  }
  close(MAT);
  last;
}

if (!%materials_rgb) { warn "(warning: did not find Materials_rgb.db)"; }

sub mat_to_rgb($$) {
  my ($material, $groups) = (@_);
  my $rgb;
  if (my ($r, $g, $b) = $material =~ /^r(\d+)g(\d+)b(\d+)(a\d+)?$/) {
    # for models from Tin Chung (from Kevin McGrath), for f18 fs game
    $rgb = sprintf("\"rgb=(%.3f %.3f %.3f)\"", $r/255, $g/255, $b/255);
  } elsif ($rgb = $materials_rgb{$material}) {
  } elsif ($rgb = $materials_rgb{$groups}) {
  } elsif ($material eq "atlasTextureMap") {
    $rgb = $materials_rgb{"unknown"} or croak "?";
  } else {
    warn "material $material (group $groups) not found!\n" unless $warning{"$material $groups"}++;
    $rgb = $materials_rgb{"unknown"} or croak "?";
  }
  $rgb;
};

1;
