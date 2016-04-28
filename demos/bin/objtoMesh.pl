#!/usr/bin/perl -s
# Takes Wavefront obj ascii file, and converts it to a mesh

# switch: -smooth_from_normals
# switch: -no_sharp_edges
# switch: -debug

# I've found that for shark and sandal, the normals match the bad smoothing groups.

use strict;
use vars qw/$smooth_from_normals $no_sharp_edges $debug/;

binmode(STDOUT);

BEGIN { push(@INC, "$ENV{HOME}/other/perl"); }
# require("materials_lib.pl");
use Materials_lib;

my $k_no_smoothing = "ZZ";
my $smoothing_group = $k_no_smoothing;
my $k_default_group = "default";
my $groups = $k_default_group;
my $material = "nomaterial";          # no default material

my $warnings1; my $warn_indices; my $duplicated_edge; my $ignore_lines; my $mrgbwarning;
my %materials;
my %mtlrgb;
my %enormal1; my %enormal2; my %edge_sg;

my @ar_v; my @ar_vn; my @ar_vt; my @ar_vrgb;
my $gvi = 1;

if (!$debug) { $warn_indices = 1; } # disable warning

LINE: while (<>) {
  if (/^v /) {
    my ($x, $y, $z, $com) = /^v\s+(\S+)\s+(\S+)\s+(\S+)(\s+\#\w+)?\s*$/ or die "? '$_'";
    push(@ar_v, "$x $y $z");
    if ($com) {			# parse format from Phil Dench's ply2obj
      $com =~ s/^\s+\#//; $com =~ s/\s*$//;
      length($com)==6 or die "?"; # 3 alphanumeric bytes
      my @col; for my $i (0..2) { $col[$i] = hex(substr($com, $i*2, 2))/255.0; }
      push(@ar_vrgb, sprintf("(%.4g %.4g %.4g)", @col));
    }
  } elsif (/^vt /) {
    my ($u, $v) = /^vt\s+(\S+)\s+(\S+)\s*$/ or die "? '$_'";
    push(@ar_vt, "($u $v)");
  } elsif (/^vn /) {
    my ($nx, $ny, $nz) = /^vn\s+(\S+)\s+(\S+)\s+(\S+)\s*$/ or die "? '$_'";
    push(@ar_vn, "($nx $ny $nz)");
  } elsif (/^f /) {
    if (@ar_v) {
      if (@ar_vt && @ar_vt!=@ar_v) { warn "Texture coordinates not 1-1\n"; @ar_vt=(); }
      if (@ar_vn && @ar_vn!=@ar_v) { warn "Normal coordinates not 1-1\n"; @ar_vn=(); }
      if (@ar_vrgb && @ar_vrgb!=@ar_v) { warn "RGB coordinates not 1-1\n"; @ar_vrgb=(); }
      for my $i (0 .. @ar_v-1) {
        my $info = " {";
        if (@ar_vn) { $info .= " normal=$ar_vn[$i]"; }
        if (@ar_vrgb) { $info .= " rgb=$ar_vrgb[$i]"; }
        if (@ar_vt) { $info .= " uv=$ar_vt[$i]"; }
        $info .= "}";
        $info =~ s/ {}//;
        $info =~ s/{ /{/;
        # my $nv = $i+1;
        my $nv = $gvi++;
        print "Vertex $nv  $ar_v[$i]$info\n";
      }
      @ar_v = ();
    }
    s/^f //; my @fullvertices = split(' ');
    my @vertices = @fullvertices;
    for (@vertices) { s@/.*$@@; }
    my $lastv = $vertices[$#vertices];
    for (@vertices) {
      if ($_<0) { $_ = -$_; warn "negative vertex indices" unless $warnings1++; }
    }
    for (@vertices) {
      my $curv = $_;
      my $edge = "$lastv $curv";
      if (defined($edge_sg{$edge})) {
	warn "Edge '$edge' already defined\n" unless $duplicated_edge++;
	next LINE;
      }
      $lastv = $curv;
    }
    if ($smooth_from_normals) {
      for (0..$#vertices) {
	my ($curv, $texti, $normi) = $fullvertices[$_] =~ m@^(\d+)/(\d*)/(\d*)$@ or die "? '$_'";
	my $lastv = $vertices[($_+@vertices-1)%@vertices];
	my $nextv = $vertices[($_+1)%@vertices];
	# warn "enor1 ($lastv $curv)=$normi\n";
	# warn "enor2 ($curv $nextv)=$normi\n";
	$enormal1{"$lastv $curv"} = $normi;
	$enormal2{"$curv $nextv"} = $normi;
      }
    } else {
      for my $v (@fullvertices) {
	if ($v=~m@/@) { warn "normal/texture indices ignored***\n" unless $warn_indices++; }
      }
    }
    my $rgb;
    if (@ar_vrgb) {
      $rgb = "";
    } elsif ($mtlrgb{$material}) {
      $rgb = $mtlrgb{$material};
    } else {
      # $rgb = (" " . &materials_lib'mat_to_rgb($material, $groups)); # emacs formatting ');
      $rgb = (" " . Materials_lib::mat_to_rgb($material, $groups));
    }
    $materials{$material}++;
    $lastv = $vertices[$#vertices];
    for my $curv (@vertices) {
      my $edge = "$lastv $curv";
      $edge_sg{$edge} = $smoothing_group;
      $lastv = $curv;
    }
    print "Face 0  @vertices {mat=\"$material\" groups=\"$groups\"$rgb}\n";
  } elsif (/^s /) {
    if (/^s\s+off\s*$/) {
      $smoothing_group = $k_no_smoothing;
    } else {
      ($smoothing_group) = /^s\s+(\w+)\s*$/ or die "? '$_'";
      # $smoothing_group>=0 or die "?";
    }
  } elsif (/^g/) {
    if (/^g$/) { $groups = $k_default_group; next; }
    ($groups) = /^g\s+(.*)\s*$/ or die "?";
  } elsif (/^usemtl /) {
    ($material) = /^usemtl\s+(\S+)\s*$/ or die "?";
  } elsif (/^l /) {
    warn "Ignoring lines\n" unless $ignore_lines++;
  } elsif (/^\#MRGB /) {        # ZBrush vertex colors
    if (@ar_v && !$mrgbwarning++) { warn "#MRGB lines should be moved prior to vertices\n"; }
    # The following MRGB block contains ZBrush Vertex Color (Polypaint) and masking output as 4 hexadecimal values per vertex. The vertex color format is MMRRGGBB with up to 64 entries per MRGB line.
    #MRGB fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffefefefffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffefefefffffffffffffffffffefefefffffffffffffffffffffffffffffffffffffffffffefefeffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff
    # ...
    # End of MRGB block
    s/^#MRGB // or die "?";
    while (s/([0-9a-f][0-9a-f])([0-9a-f][0-9a-f])([0-9a-f][0-9a-f])//) {
      my @rgb = ($1, $2, $3);
      for my $ch (@rgb) {
        $ch = ord(pack("H2", $ch))/255.0;
      }
      # push @ar_vrgb, "@rgb";
      push @ar_vrgb, sprintf("%g %g %g", @rgb);
      # if ($lnum++>10) { print @ar_vrgb; exit 1; }
    }
  } elsif (/^\#/) {		# ignore comment
  } elsif (/^\s*$/) {		# ignore blank lines
  } elsif (/^mtllib /) {
    (my $filename) = /^mtllib (\S+)$/ or die "?";
    $filename =~ /\.mtl$/ or die "$filename not *.mtl";
    ## Example file1:
    # newmtl 01_-_Default
    # Ka  0.6 0.6 0.6
    # Kd  0.6 0.6 0.6
    # Ks  0.9 0.9 0.9
    # d  1.0
    # Ns  0.0
    # illum 2
    # map_Kd TextureDouble_A.png
    ## Example file2:
    # # Material Count: 4
    # 
    # newmtl Material
    # Ns 96.078431
    # Ka 1.000000 1.000000 1.000000
    # Kd 0.045653 0.136372 0.640000
    # Ks 0.500000 0.500000 0.500000
    # Ke 0.000000 0.000000 0.000000
    # Ni 1.000000
    # d 1.000000
    # illum 2
    # 
    # newmtl Material.005
    # Ns 96.078431
    # Ka 1.000000 1.000000 1.000000
    # Kd 0.156315 0.800000 0.754417
    # Ks 0.500000 0.500000 0.500000
    # Ke 0.000000 0.000000 0.000000
    # Ni 1.000000
    # d 1.000000
    # illum 2
    #...
    open(MTLLIB, $filename) or die "mtllib file $filename not found";
    my $material;
    while (<MTLLIB>) {
      if (/^newmtl (\S+)$/) { $material = $1; next; }
      if (/^Kd\s+(\S+)\s+(\S+)\s+(\S+)\s*$/) {
        die "?" unless $material && !$mtlrgb{$material};
        $mtlrgb{$material} = " rgb=($1 $2 $3)";
      }
    }
    close(MTLLIB);
  } elsif (/^o /) {
    chop; warn "line '$_' ignored\n" if $debug;
    # chop; warn "Adding new object based on line '$_'\n";
    # printf "o 1 1 0\n";
    printf("# %s\n", $_);
  } else {
    chop; die "could not parse line '$_'";
  }
}

if (!$no_sharp_edges && !@ar_vn) {
  warn "Now looking for sharp edges\n";
  my $num_boundary_edges = 0; my $num_internal_edges = 0; my $num_sharp_edges = 0;
  # while (($edge, $sg1) = each %edge_sg) {
  for my $edge (keys %edge_sg) {
    my $sg1 = $edge_sg{$edge};
    my ($a, $b) = $edge =~ /^(\d+) (\d+)$/ or die "?";
    my $opposite = "$b $a"; my $sg2 = $edge_sg{$opposite};
    if (!defined($sg2)) { $num_boundary_edges++; next; }
    next if $a>$b;
    $num_internal_edges++;
    if ($smooth_from_normals) {
      my $efn1 = $enormal1{$edge};
      my $efn2 = $enormal2{$edge};
      my $eon1 = $enormal1{$opposite};
      my $eon2 = $enormal2{$opposite};
      next if ($efn1==$eon2 && $efn2==$eon1);
    } else {
      # warn "sg1=$sg1 sg2=$sg2\n";
      next if ($sg1 ne $k_no_smoothing && $sg1 eq $sg2);
    }
    printf("Edge $a $b {sharp}\n"); $num_sharp_edges++;
  }
  printf("# found edges: total=%d boundary=%d sharp=%d\n",
	 $num_boundary_edges+$num_internal_edges, $num_boundary_edges, $num_sharp_edges);
}

for (sort keys %materials) {
  printf("# material \"%s\" %s\n", $_, $materials{$_});
}

warn "Found $duplicated_edge duplicated edges\n" if $duplicated_edge;
