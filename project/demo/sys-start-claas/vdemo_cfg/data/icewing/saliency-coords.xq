declare function local:transform2D($p as xs:double*, $T as xs:double*) as xs:double*
{
	for $i in 1 to 3
	return sum(for $j in 1 to 3
		return $p[$j] * $T[(($i - 1) * 3) + $j])
};

declare function local:transformRegion($n, $T as xs:double*) as node()
{
	for $r in $n
	let $wp := local:transform2D( ($r/@cx, $r/@cy, 1), $T )
	return element region { $r/@* except ($r/@cx, $r/@cy) |
		attribute cx { $wp[1] } | attribute cy { $wp[2] } |
		attribute cz { 0 } |
		attribute imageX { $r/@cx } | attribute imageY { $r/@cy } } 
};

for $n in doc("saliency.xml")/saliencies
let $width := $n/@width
let $height := $n/@height
(: transform from (relative) camera coordinates to world :)
let $T := (( 132.257     0.819145 -65.6292 ),
    	 	  (   1.12297 -99.2219    90.4504 ))
return
	<saliencies width="{$width}" height="{$height}">
        {
            for $n in $n/*[name() != "region"]
            return $n
        }
	{ 
		for $r in $n/region
                let $x := $r/@cx
                let $y := $r/@cy
					 let $rx := $x div $width
                let $ry := $y div $height
                let $wp := local:transform2D(($rx, $ry, 1), $T)
                return 
                    <point>
                        <coord ref="image" kind="absolute" x="{$x}" y="{$y}"/>
                        <coord ref="image" kind="relative" x="{$rx}" y="{$ry}"/>
                        <coord ref="world" kind="absolute" unit="cm" x="{$wp[1]}"
                            y="{$wp[2]}" z="3"/>
                        { element saliency { $r/@* except ( $x, $y ), $r/*} }
                    </point>
	}
	</saliencies>
