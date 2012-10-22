#!/usr/bin/perl -w
my @x;
my @y;
my @z;
my $count;
while(<>){
    next if /^bad/;
    $count++;
    chomp;
    /(.*?) (.*?) (.*?)$/;
    push @x,$1;
    push @y,$2;
    push @z,$3;
}
my $xx;
my $yy;
my $zz;
($xx+=$_) for @x;
$xx=sprintf ("%.2f",$xx/$count);
($yy+=$_) for @y;
$yy=sprintf ("%.2f",$yy/$count);
($zz+=$_) for @z;
$zz=sprintf ("%.2f",$zz/$count);
#print "avg:x=$xx "."y=$yy "."z=$zz"."\n";
my @dist;
my $i=0;
my @index_count;
while($i<$count){
    my $this_dist;
    $this_dist=sqrt(($x[$i]-$xx)*($x[$i]-$xx)+($y[$i]-$yy)*($y[$i]-$yy)+($z[$i]-$zz)*($z[$i]-$zz));
    foreach(1..40){
	if ($this_dist<$_){
	    $index_count[$_]++;
	}
    }
    #print $this_dist."\n";
    $i++;
}
my $temp_count=0;
foreach (@index_count){
    print ($temp_count++);
    print "\t,".($_/$count)."\n";
}
