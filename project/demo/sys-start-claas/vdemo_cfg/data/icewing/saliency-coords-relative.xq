for $n in doc("saliency.xml")/saliencies
let $width := $n/@width
let $height := $n/@height
let $relsize := 1.0 div $width
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
                return 
                    <point>
                        <coord ref="image" kind="absolute" x="{$x}" y="{$y}" w="1" h="1"/>
                        <coord ref="image" kind="relative" x="{$rx}" y="{$ry}" w="{$relsize}" h="{$relsize}"/>
                        { element saliency { $r/@* except ( $x, $y ), $r/*} }
                    </point>
	}
	</saliencies>
