#!/usr/bin/perl

use strict;
use warnings;

use constant OFFSET => 4.0;

# Maze definition
my @maze;
my $maze_filename = './maze.txt';
open(my $fh, '<', $maze_filename) or die "Could not open file '$maze_filename' $!";

my $row_cnt = 0;
my $col_cnt = 0;
while (my $read = read $fh, my $char, 1) {
  if ($char eq '#' || $char eq ' ') {
    $maze[$row_cnt][$col_cnt++] = $char;
  } elsif ($char eq "\n") {
    $row_cnt++;
    $col_cnt = 0;
  } else {
    die "Unknown character: '$char'";
  }
}

close $fh;

my $world_filename = './src/jhonny5/worlds/maze.world';
open($fh, '>', $world_filename) or die "Could not open file '$world_filename' $!";

print $fh <<'END_MESSAGE';
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="maze_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- MAZE START -->
END_MESSAGE

# Generate maze
for (my $i = 0; $i < @maze; $i++) {
  my $row_ref = $maze[$i];
  for (my $j = 0; $j < @$row_ref; $j++) {
    if (@$row_ref[$j] eq '#') {
      print $fh "<include>\n";
      print $fh "\t<uri>model://unit_box</uri>\n";
      print $fh "\t<pose>", $i * OFFSET - OFFSET, " ", $j * OFFSET - OFFSET, " 0 0 0 0</pose>\n";
      print $fh "</include>\n";
    }
  }
}
print $fh "\t\t<!-- MAZE END -->\n";
# End of maze generation

print $fh "\t</world>\n";
print $fh "</sdf>\n";

close $fh;
