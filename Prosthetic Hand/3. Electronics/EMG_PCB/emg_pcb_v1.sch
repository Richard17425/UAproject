<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.6.2">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="10" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="26" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="con-molex" urn="urn:adsk.eagle:library:165">
<description>&lt;b&gt;Molex Connectors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="53047-07" urn="urn:adsk.eagle:footprint:8078147/1" library_version="5">
<description>&lt;b&gt;1.25mm Pitch PicoBlade™ Wire-to-Board Header, Vertical, with Friction Lock, 7 Circuits&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/530470710_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<wire x1="-5.15" y1="-1.5" x2="5.15" y2="-1.5" width="0.2032" layer="21"/>
<wire x1="5.15" y1="-1.5" x2="5.15" y2="1.5" width="0.2032" layer="21"/>
<wire x1="5.15" y1="1.5" x2="-5.15" y2="1.5" width="0.2032" layer="21"/>
<wire x1="-5.15" y1="1.5" x2="-5.15" y2="-1.5" width="0.2032" layer="21"/>
<wire x1="-5.125" y1="-0.25" x2="-4.75" y2="-0.25" width="0.0508" layer="21"/>
<wire x1="-4.75" y1="-0.25" x2="-4.75" y2="-1.25" width="0.0508" layer="21"/>
<wire x1="4.75" y1="-0.25" x2="5.125" y2="-0.25" width="0.0508" layer="21"/>
<wire x1="4.75" y1="-0.25" x2="4.75" y2="-1.25" width="0.0508" layer="21"/>
<wire x1="-5.125" y1="0.375" x2="-4.75" y2="0.375" width="0.0508" layer="21"/>
<wire x1="4.75" y1="0.375" x2="5.125" y2="0.375" width="0.0508" layer="21"/>
<wire x1="-4.75" y1="0.375" x2="-4.75" y2="1.125" width="0.0508" layer="21"/>
<wire x1="-4.75" y1="1.125" x2="4.75" y2="1.125" width="0.0508" layer="21"/>
<wire x1="4.75" y1="1.125" x2="4.75" y2="0.375" width="0.0508" layer="21"/>
<wire x1="4.75" y1="1.125" x2="5" y2="1.375" width="0.0508" layer="21"/>
<wire x1="-4.75" y1="1.125" x2="-5" y2="1.375" width="0.0508" layer="21"/>
<pad name="1" x="3.75" y="0.45" drill="0.5" diameter="0.6984" shape="long" rot="R270"/>
<pad name="2" x="2.5" y="0.45" drill="0.5" diameter="0.6984" shape="long" rot="R270"/>
<pad name="3" x="1.25" y="0.45" drill="0.5" diameter="0.6984" shape="long" rot="R270"/>
<pad name="4" x="0" y="0.45" drill="0.5" diameter="0.6984" shape="long" rot="R270"/>
<pad name="5" x="-1.25" y="0.45" drill="0.5" diameter="0.6984" shape="long" rot="R270"/>
<pad name="6" x="-2.5" y="0.45" drill="0.5" diameter="0.6984" shape="long" rot="R270"/>
<pad name="7" x="-3.75" y="0.45" drill="0.5" diameter="0.6984" shape="long" rot="R270"/>
<text x="-5" y="1.75" size="1.27" layer="25">&gt;NAME</text>
<text x="-4.5" y="-3.25" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-5.125" y1="-1.5" x2="5.125" y2="-1.125" layer="21"/>
</package>
<package name="53048-07" urn="urn:adsk.eagle:footprint:8078189/1" library_version="5">
<description>&lt;b&gt;1.25mm Pitch PicoBlade™ Header, Right Angle, 7 Circuits&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/530480710_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<wire x1="-5.15" y1="-2.25" x2="-4.625" y2="-2.25" width="0.2032" layer="21"/>
<wire x1="4.625" y1="-2.25" x2="5.15" y2="-2.25" width="0.2032" layer="21"/>
<wire x1="5.15" y1="-2.25" x2="5.15" y2="3.125" width="0.2032" layer="21"/>
<wire x1="5.15" y1="3.125" x2="5" y2="3.125" width="0.2032" layer="21"/>
<wire x1="5" y1="3.125" x2="-5" y2="3.125" width="0.2032" layer="21"/>
<wire x1="-5" y1="3.125" x2="-5.15" y2="3.125" width="0.2032" layer="21"/>
<wire x1="-5.15" y1="3.125" x2="-5.15" y2="-2.25" width="0.2032" layer="21"/>
<wire x1="-5.125" y1="1.5" x2="-4.625" y2="1.5" width="0.0508" layer="21"/>
<wire x1="-4.625" y1="1.5" x2="-4.375" y2="1.5" width="0.0508" layer="21"/>
<wire x1="-4.375" y1="1.5" x2="-4.375" y2="0.625" width="0.0508" layer="21"/>
<wire x1="-4.375" y1="0.625" x2="4.375" y2="0.625" width="0.0508" layer="21"/>
<wire x1="4.375" y1="1.5" x2="4.625" y2="1.5" width="0.0508" layer="21"/>
<wire x1="4.625" y1="1.5" x2="5.125" y2="1.5" width="0.0508" layer="21"/>
<wire x1="4.375" y1="1.5" x2="4.375" y2="0.625" width="0.0508" layer="21"/>
<wire x1="-4.625" y1="-1" x2="4.625" y2="-1" width="0.2032" layer="51"/>
<wire x1="-4" y1="-1.5" x2="-4.125" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="-2.75" y1="-1.5" x2="-2.875" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="-2.875" y1="-1.625" x2="-3.375" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="-3.5" y1="-1.5" x2="-3.375" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="-1.5" y1="-1.5" x2="-1.625" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="-1.625" y1="-1.625" x2="-2.125" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="-2.25" y1="-1.5" x2="-2.125" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="-0.25" y1="-1.5" x2="-0.375" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="-0.375" y1="-1.625" x2="-0.875" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="-1" y1="-1.5" x2="-0.875" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="1" y1="-1.5" x2="0.875" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="0.875" y1="-1.625" x2="0.375" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="0.25" y1="-1.5" x2="0.375" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="2.25" y1="-1.5" x2="2.125" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="2.125" y1="-1.625" x2="1.625" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="1.5" y1="-1.5" x2="1.625" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="3.5" y1="-1.5" x2="3.375" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="3.375" y1="-1.625" x2="2.875" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="2.75" y1="-1.5" x2="2.875" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="4.625" y1="-1.625" x2="4.125" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="4" y1="-1.5" x2="4.125" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="-4.125" y1="-1.625" x2="-4.625" y2="-1.625" width="0.2032" layer="51"/>
<wire x1="-5.125" y1="-1" x2="-4.625" y2="-1" width="0.2032" layer="21"/>
<wire x1="-4.625" y1="-1" x2="-4.625" y2="-2.25" width="0.2032" layer="21"/>
<wire x1="4.625" y1="-1" x2="5.125" y2="-1" width="0.2032" layer="21"/>
<wire x1="4.625" y1="-1" x2="4.625" y2="-2.25" width="0.2032" layer="21"/>
<wire x1="-4.625" y1="1.5" x2="-4.625" y2="2.75" width="0.0508" layer="21"/>
<wire x1="-4.625" y1="2.75" x2="4.625" y2="2.75" width="0.0508" layer="21"/>
<wire x1="4.625" y1="2.75" x2="4.625" y2="1.5" width="0.0508" layer="21"/>
<wire x1="-4.625" y1="2.75" x2="-5" y2="3.125" width="0.0508" layer="21"/>
<wire x1="4.625" y1="2.75" x2="5" y2="3.125" width="0.0508" layer="21"/>
<wire x1="-3.875" y1="1.5" x2="-3.75" y2="2" width="0.2032" layer="21"/>
<wire x1="-3.75" y1="2" x2="-3.625" y2="1.5" width="0.2032" layer="21"/>
<wire x1="-2.625" y1="1.5" x2="-2.5" y2="2" width="0.2032" layer="21"/>
<wire x1="-2.5" y1="2" x2="-2.375" y2="1.5" width="0.2032" layer="21"/>
<wire x1="-1.375" y1="1.5" x2="-1.25" y2="2" width="0.2032" layer="21"/>
<wire x1="-1.25" y1="2" x2="-1.125" y2="1.5" width="0.2032" layer="21"/>
<wire x1="-0.125" y1="1.5" x2="0" y2="2" width="0.2032" layer="21"/>
<wire x1="0" y1="2" x2="0.125" y2="1.5" width="0.2032" layer="21"/>
<wire x1="1.125" y1="1.5" x2="1.25" y2="2" width="0.2032" layer="21"/>
<wire x1="1.25" y1="2" x2="1.375" y2="1.5" width="0.2032" layer="21"/>
<wire x1="2.375" y1="1.5" x2="2.5" y2="2" width="0.2032" layer="21"/>
<wire x1="2.5" y1="2" x2="2.625" y2="1.5" width="0.2032" layer="21"/>
<wire x1="3.625" y1="1.5" x2="3.75" y2="2" width="0.2032" layer="21"/>
<wire x1="3.75" y1="2" x2="3.875" y2="1.5" width="0.2032" layer="21"/>
<pad name="1" x="3.75" y="-1.25" drill="0.5" diameter="0.6984" shape="long" rot="R90"/>
<pad name="2" x="2.5" y="-1.25" drill="0.5" diameter="0.6984" shape="long" rot="R90"/>
<pad name="3" x="1.25" y="-1.25" drill="0.5" diameter="0.6984" shape="long" rot="R90"/>
<pad name="4" x="0" y="-1.25" drill="0.5" diameter="0.6984" shape="long" rot="R90"/>
<pad name="5" x="-1.25" y="-1.25" drill="0.5" diameter="0.6984" shape="long" rot="R90"/>
<pad name="6" x="-2.5" y="-1.25" drill="0.5" diameter="0.6984" shape="long" rot="R90"/>
<pad name="7" x="-3.75" y="-1.25" drill="0.5" diameter="0.6984" shape="long" rot="R90"/>
<text x="-3.75" y="3.375" size="1.27" layer="25">&gt;NAME</text>
<text x="-4.75" y="-3.75" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-4" y1="-1.5" x2="-3.5" y2="-1" layer="51"/>
<rectangle x1="-2.75" y1="-1.5" x2="-2.25" y2="-1" layer="51"/>
<rectangle x1="-1.5" y1="-1.5" x2="-1" y2="-1" layer="51"/>
<rectangle x1="-0.25" y1="-1.5" x2="0.25" y2="-1" layer="51"/>
<rectangle x1="1" y1="-1.5" x2="1.5" y2="-1" layer="51"/>
<rectangle x1="2.25" y1="-1.5" x2="2.75" y2="-1" layer="51"/>
<rectangle x1="3.5" y1="-1.5" x2="4" y2="-1" layer="51"/>
<rectangle x1="-4" y1="0.625" x2="-3.5" y2="1.5" layer="21"/>
<rectangle x1="-2.75" y1="0.625" x2="-2.25" y2="1.5" layer="21"/>
<rectangle x1="-1.5" y1="0.625" x2="-1" y2="1.5" layer="21"/>
<rectangle x1="-0.25" y1="0.625" x2="0.25" y2="1.5" layer="21"/>
<rectangle x1="1" y1="0.625" x2="1.5" y2="1.5" layer="21"/>
<rectangle x1="2.25" y1="0.625" x2="2.75" y2="1.5" layer="21"/>
<rectangle x1="3.5" y1="0.625" x2="4" y2="1.5" layer="21"/>
</package>
<package name="53261-07" urn="urn:adsk.eagle:footprint:8078162/1" library_version="5">
<description>&lt;b&gt;1.25mm Pitch PicoBlade™ Header, Surface Mount, Right Angle, 7 Circuits&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/532610771_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<wire x1="-5.15" y1="-1.375" x2="-4.625" y2="-1.375" width="0.2032" layer="21"/>
<wire x1="-4.625" y1="-1.375" x2="4.625" y2="-1.375" width="0.2032" layer="21"/>
<wire x1="4.625" y1="-1.375" x2="5.15" y2="-1.375" width="0.2032" layer="21"/>
<wire x1="5.15" y1="-1.375" x2="5.15" y2="2.625" width="0.2032" layer="21"/>
<wire x1="5.15" y1="2.625" x2="5" y2="2.625" width="0.2032" layer="21"/>
<wire x1="5" y1="2.625" x2="-5" y2="2.625" width="0.2032" layer="21"/>
<wire x1="-5" y1="2.625" x2="-5.15" y2="2.625" width="0.2032" layer="21"/>
<wire x1="-5.15" y1="2.625" x2="-5.15" y2="-1.375" width="0.2032" layer="21"/>
<wire x1="-5.125" y1="1.625" x2="-4.625" y2="1.625" width="0.0508" layer="21"/>
<wire x1="-4.625" y1="1.625" x2="-4.375" y2="1.625" width="0.0508" layer="21"/>
<wire x1="-4.375" y1="1.625" x2="-4.375" y2="1" width="0.0508" layer="21"/>
<wire x1="-4.375" y1="1" x2="4.375" y2="1" width="0.0508" layer="21"/>
<wire x1="4.375" y1="1.625" x2="4.625" y2="1.625" width="0.0508" layer="21"/>
<wire x1="4.625" y1="1.625" x2="5.125" y2="1.625" width="0.0508" layer="21"/>
<wire x1="4.375" y1="1.625" x2="4.375" y2="1" width="0.0508" layer="21"/>
<wire x1="-5.125" y1="-0.75" x2="-4.625" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="-4.625" y1="-0.75" x2="-4.625" y2="-1.375" width="0.0508" layer="21"/>
<wire x1="4.625" y1="-0.75" x2="5.125" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="4.625" y1="-0.75" x2="4.625" y2="-1.375" width="0.0508" layer="21"/>
<wire x1="-4.625" y1="1.625" x2="-4.625" y2="2.25" width="0.0508" layer="21"/>
<wire x1="-4.625" y1="2.25" x2="4.625" y2="2.25" width="0.0508" layer="21"/>
<wire x1="4.625" y1="2.25" x2="4.625" y2="1.625" width="0.0508" layer="21"/>
<wire x1="-4.625" y1="2.25" x2="-5" y2="2.625" width="0.0508" layer="21"/>
<wire x1="4.625" y1="2.25" x2="5" y2="2.625" width="0.0508" layer="21"/>
<wire x1="-4" y1="-1.25" x2="-4" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="-4" y1="-0.75" x2="-3.5" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="-3.5" y1="-0.75" x2="-3.5" y2="-1.25" width="0.0508" layer="21"/>
<wire x1="-2.75" y1="-1.25" x2="-2.75" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="-2.75" y1="-0.75" x2="-2.25" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="-2.25" y1="-0.75" x2="-2.25" y2="-1.25" width="0.0508" layer="21"/>
<wire x1="-1.5" y1="-1.25" x2="-1.5" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="-1.5" y1="-0.75" x2="-1" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="-1" y1="-0.75" x2="-1" y2="-1.25" width="0.0508" layer="21"/>
<wire x1="-0.25" y1="-1.25" x2="-0.25" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="-0.25" y1="-0.75" x2="0.25" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="0.25" y1="-0.75" x2="0.25" y2="-1.25" width="0.0508" layer="21"/>
<wire x1="1" y1="-1.25" x2="1" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="1" y1="-0.75" x2="1.5" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="1.5" y1="-0.75" x2="1.5" y2="-1.25" width="0.0508" layer="21"/>
<wire x1="2.25" y1="-1.25" x2="2.25" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="2.25" y1="-0.75" x2="2.75" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="2.75" y1="-0.75" x2="2.75" y2="-1.25" width="0.0508" layer="21"/>
<wire x1="3.5" y1="-1.25" x2="3.5" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="3.5" y1="-0.75" x2="4" y2="-0.75" width="0.0508" layer="21"/>
<wire x1="4" y1="-0.75" x2="4" y2="-1.25" width="0.0508" layer="21"/>
<wire x1="-5.25" y1="2.25" x2="-7.125" y2="2.25" width="0.2032" layer="51"/>
<wire x1="-7.125" y1="2.25" x2="-7.125" y2="-0.75" width="0.2032" layer="51"/>
<wire x1="-7.125" y1="-0.75" x2="-5.25" y2="-0.75" width="0.2032" layer="51"/>
<wire x1="5.25" y1="-0.75" x2="7.125" y2="-0.75" width="0.2032" layer="51"/>
<wire x1="7.125" y1="-0.75" x2="7.125" y2="2.25" width="0.2032" layer="51"/>
<wire x1="7.125" y1="2.25" x2="5.25" y2="2.25" width="0.2032" layer="51"/>
<smd name="1" x="3.75" y="-2.5" dx="0.8" dy="2" layer="1"/>
<smd name="2" x="2.5" y="-2.5" dx="0.8" dy="2" layer="1"/>
<smd name="3" x="1.25" y="-2.5" dx="0.8" dy="2" layer="1"/>
<smd name="4" x="0" y="-2.5" dx="0.8" dy="2" layer="1"/>
<smd name="5" x="-1.25" y="-2.5" dx="0.8" dy="2" layer="1"/>
<smd name="6" x="-2.5" y="-2.5" dx="0.8" dy="2" layer="1"/>
<smd name="7" x="-3.75" y="-2.5" dx="0.8" dy="2" layer="1"/>
<smd name="S1" x="6.25" y="0.625" dx="2.1" dy="3" layer="1"/>
<smd name="S2" x="-6.25" y="0.625" dx="2.1" dy="3" layer="1"/>
<text x="-5" y="2.875" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.625" y="-0.5" size="1.27" layer="27">&gt;VALUE</text>
<text x="5.75" y="-0.25" size="1.9304" layer="51">1</text>
<rectangle x1="-4" y1="1" x2="-3.5" y2="1.875" layer="21"/>
<rectangle x1="-2.75" y1="1" x2="-2.25" y2="1.875" layer="21"/>
<rectangle x1="-1.5" y1="1" x2="-1" y2="1.875" layer="21"/>
<rectangle x1="-0.25" y1="1" x2="0.25" y2="1.875" layer="21"/>
<rectangle x1="1" y1="1" x2="1.5" y2="1.875" layer="21"/>
<rectangle x1="2.25" y1="1" x2="2.75" y2="1.875" layer="21"/>
<rectangle x1="3.5" y1="1" x2="4" y2="1.875" layer="21"/>
</package>
<package name="53398-07" urn="urn:adsk.eagle:footprint:8078176/1" library_version="5">
<description>&lt;b&gt;1.25mm Pitch PicoBlade™ Header, Surface Mount, Vertical, 7 Circuits&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/533980771_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<wire x1="-5.15" y1="-1.375" x2="-4.375" y2="-1.375" width="0.2032" layer="21"/>
<wire x1="-4.375" y1="-1.375" x2="4.375" y2="-1.375" width="0.0508" layer="21"/>
<wire x1="4.375" y1="-1.375" x2="5.15" y2="-1.375" width="0.2032" layer="21"/>
<wire x1="5.15" y1="-1.375" x2="5.15" y2="2.125" width="0.2032" layer="21"/>
<wire x1="5.15" y1="2.125" x2="5" y2="2.125" width="0.2032" layer="21"/>
<wire x1="5" y1="2.125" x2="-5" y2="2.125" width="0.2032" layer="21"/>
<wire x1="-5" y1="2.125" x2="-5.15" y2="2.125" width="0.2032" layer="21"/>
<wire x1="-5.15" y1="2.125" x2="-5.15" y2="-1.375" width="0.2032" layer="21"/>
<wire x1="-5.125" y1="1" x2="-4.625" y2="1" width="0.0508" layer="21"/>
<wire x1="4.625" y1="1" x2="5.125" y2="1" width="0.0508" layer="21"/>
<wire x1="-5.125" y1="-0.25" x2="-4.625" y2="-0.25" width="0.0508" layer="21"/>
<wire x1="-4.625" y1="-0.25" x2="-4.625" y2="-1" width="0.0508" layer="21"/>
<wire x1="4.625" y1="-0.25" x2="5.125" y2="-0.25" width="0.0508" layer="21"/>
<wire x1="4.625" y1="-0.25" x2="4.625" y2="-1" width="0.0508" layer="21"/>
<wire x1="-4.625" y1="1" x2="-4.625" y2="1.75" width="0.0508" layer="21"/>
<wire x1="-4.625" y1="1.75" x2="4.625" y2="1.75" width="0.0508" layer="21"/>
<wire x1="4.625" y1="1.75" x2="4.625" y2="1" width="0.0508" layer="21"/>
<wire x1="-4.625" y1="1.75" x2="-5" y2="2.125" width="0.0508" layer="21"/>
<wire x1="4.625" y1="1.75" x2="5" y2="2.125" width="0.0508" layer="21"/>
<wire x1="-5.25" y1="2.125" x2="-7.125" y2="2.125" width="0.2032" layer="51"/>
<wire x1="-7.125" y1="2.125" x2="-7.125" y2="-0.75" width="0.2032" layer="51"/>
<wire x1="-7.125" y1="-0.75" x2="-5.25" y2="-0.75" width="0.2032" layer="51"/>
<wire x1="5.25" y1="-0.75" x2="7.125" y2="-0.75" width="0.2032" layer="51"/>
<wire x1="7.125" y1="-0.75" x2="7.125" y2="2.125" width="0.2032" layer="51"/>
<wire x1="7.125" y1="2.125" x2="5.25" y2="2.125" width="0.2032" layer="51"/>
<wire x1="-5.125" y1="-1" x2="-4.625" y2="-1" width="0.0508" layer="21"/>
<wire x1="-4.625" y1="-1" x2="-4.375" y2="-1" width="0.0508" layer="21"/>
<wire x1="-4.375" y1="-1" x2="-4.375" y2="-1.375" width="0.0508" layer="21"/>
<wire x1="4.625" y1="-1" x2="5.125" y2="-1" width="0.0508" layer="21"/>
<wire x1="4.375" y1="-1" x2="4.625" y2="-1" width="0.0508" layer="21"/>
<wire x1="4.375" y1="-1" x2="4.375" y2="-1.375" width="0.0508" layer="21"/>
<smd name="1" x="3.75" y="-2.5" dx="0.8" dy="1.8" layer="1"/>
<smd name="2" x="2.5" y="-2.5" dx="0.8" dy="1.8" layer="1"/>
<smd name="3" x="1.25" y="-2.5" dx="0.8" dy="1.8" layer="1"/>
<smd name="4" x="0" y="-2.5" dx="0.8" dy="1.8" layer="1"/>
<smd name="5" x="-1.25" y="-2.5" dx="0.8" dy="1.8" layer="1"/>
<smd name="6" x="-2.5" y="-2.5" dx="0.8" dy="1.8" layer="1"/>
<smd name="7" x="-3.75" y="-2.5" dx="0.8" dy="1.8" layer="1"/>
<smd name="S1" x="6.25" y="0.625" dx="2.1" dy="3" layer="1"/>
<smd name="S2" x="-6.25" y="0.625" dx="2.1" dy="3" layer="1"/>
<text x="-4.875" y="2.375" size="1.27" layer="25">&gt;NAME</text>
<text x="-4.375" y="-1" size="1.27" layer="27">&gt;VALUE</text>
<text x="5.625" y="-0.25" size="1.9304" layer="51">1</text>
<rectangle x1="-4" y1="0.375" x2="-3.5" y2="1" layer="21"/>
<rectangle x1="-2.75" y1="0.375" x2="-2.25" y2="1" layer="21"/>
<rectangle x1="-1.5" y1="0.375" x2="-1" y2="1" layer="21"/>
<rectangle x1="-0.25" y1="0.375" x2="0.25" y2="1" layer="21"/>
<rectangle x1="1" y1="0.375" x2="1.5" y2="1" layer="21"/>
<rectangle x1="2.25" y1="0.375" x2="2.75" y2="1" layer="21"/>
<rectangle x1="3.5" y1="0.375" x2="4" y2="1" layer="21"/>
</package>
<package name="22-23-2071" urn="urn:adsk.eagle:footprint:8078264/1" library_version="5">
<description>&lt;b&gt;KK® 254 Solid Header, Vertical, with Friction Lock, 7 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022232071_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<wire x1="-8.89" y1="3.175" x2="8.89" y2="3.175" width="0.254" layer="21"/>
<wire x1="8.89" y1="3.175" x2="8.89" y2="1.27" width="0.254" layer="21"/>
<wire x1="8.89" y1="1.27" x2="8.89" y2="-3.175" width="0.254" layer="21"/>
<wire x1="8.89" y1="-3.175" x2="-8.89" y2="-3.175" width="0.254" layer="21"/>
<wire x1="-8.89" y1="-3.175" x2="-8.89" y2="1.27" width="0.254" layer="21"/>
<wire x1="-8.89" y1="1.27" x2="-8.89" y2="3.175" width="0.254" layer="21"/>
<wire x1="-8.89" y1="1.27" x2="8.89" y2="1.27" width="0.254" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="1" shape="long" rot="R90"/>
<pad name="2" x="-5.08" y="0" drill="1" shape="long" rot="R90"/>
<pad name="3" x="-2.54" y="0" drill="1" shape="long" rot="R90"/>
<pad name="4" x="0" y="0" drill="1" shape="long" rot="R90"/>
<pad name="5" x="2.54" y="0" drill="1" shape="long" rot="R90"/>
<pad name="6" x="5.08" y="0" drill="1" shape="long" rot="R90"/>
<pad name="7" x="7.62" y="0" drill="1" shape="long" rot="R90"/>
<text x="-8.89" y="3.81" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-8.89" y="-5.08" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="53047-07" urn="urn:adsk.eagle:package:8078516/1" type="box" library_version="5">
<description>&lt;b&gt;1.25mm Pitch PicoBlade™ Wire-to-Board Header, Vertical, with Friction Lock, 7 Circuits&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/530470710_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<packageinstances>
<packageinstance name="53047-07"/>
</packageinstances>
</package3d>
<package3d name="53048-07" urn="urn:adsk.eagle:package:8078561/1" type="box" library_version="5">
<description>&lt;b&gt;1.25mm Pitch PicoBlade™ Header, Right Angle, 7 Circuits&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/530480710_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<packageinstances>
<packageinstance name="53048-07"/>
</packageinstances>
</package3d>
<package3d name="53261-07" urn="urn:adsk.eagle:package:8078532/1" type="box" library_version="5">
<description>&lt;b&gt;1.25mm Pitch PicoBlade™ Header, Surface Mount, Right Angle, 7 Circuits&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/532610771_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<packageinstances>
<packageinstance name="53261-07"/>
</packageinstances>
</package3d>
<package3d name="53398-07" urn="urn:adsk.eagle:package:8078546/1" type="box" library_version="5">
<description>&lt;b&gt;1.25mm Pitch PicoBlade™ Header, Surface Mount, Vertical, 7 Circuits&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/533980771_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<packageinstances>
<packageinstance name="53398-07"/>
</packageinstances>
</package3d>
<package3d name="22-23-2071" urn="urn:adsk.eagle:package:8078638/1" type="box" library_version="5">
<description>&lt;b&gt;KK® 254 Solid Header, Vertical, with Friction Lock, 7 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022232071_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<packageinstances>
<packageinstance name="22-23-2071"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="MV" urn="urn:adsk.eagle:symbol:6783/2" library_version="5">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<text x="-0.762" y="1.397" size="1.778" layer="96">&gt;VALUE</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
<symbol name="M" urn="urn:adsk.eagle:symbol:6785/2" library_version="5">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="53?-07" urn="urn:adsk.eagle:component:8078949/3" prefix="X" library_version="5">
<description>&lt;b&gt;CONNECTOR&lt;/b&gt;&lt;p&gt;
wire to board 1.25 mm (.049 inch) pitch header</description>
<gates>
<gate name="-1" symbol="MV" x="0" y="7.62" addlevel="always" swaplevel="1"/>
<gate name="-2" symbol="M" x="0" y="5.08" addlevel="always" swaplevel="1"/>
<gate name="-3" symbol="M" x="0" y="2.54" addlevel="always" swaplevel="1"/>
<gate name="-4" symbol="M" x="0" y="0" addlevel="always" swaplevel="1"/>
<gate name="-5" symbol="M" x="0" y="-2.54" addlevel="always" swaplevel="1"/>
<gate name="-6" symbol="M" x="0" y="-5.08" addlevel="always" swaplevel="1"/>
<gate name="-7" symbol="M" x="0" y="-7.62" addlevel="always" swaplevel="1"/>
</gates>
<devices>
<device name="047" package="53047-07">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8078516/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="53047-0710-C" constant="no"/>
<attribute name="OC_FARNELL" value="9732870" constant="no"/>
<attribute name="OC_NEWARK" value="11M0149" constant="no"/>
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="048" package="53048-07">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8078561/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="53048-0710" constant="no"/>
<attribute name="OC_FARNELL" value="9732934" constant="no"/>
<attribute name="OC_NEWARK" value="98K9835" constant="no"/>
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="261" package="53261-07">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8078532/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="1125361" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
<device name="398" package="53398-07">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8078546/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="9786368" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="22-23-2071" urn="urn:adsk.eagle:component:8078933/3" prefix="X" library_version="5">
<description>.100" (2.54mm) Center Header - 7 Pin</description>
<gates>
<gate name="-1" symbol="MV" x="0" y="7.62" addlevel="always" swaplevel="1"/>
<gate name="-2" symbol="M" x="0" y="5.08" addlevel="always" swaplevel="1"/>
<gate name="-3" symbol="M" x="0" y="2.54" addlevel="always" swaplevel="1"/>
<gate name="-4" symbol="M" x="0" y="0" addlevel="always" swaplevel="1"/>
<gate name="-5" symbol="M" x="0" y="-2.54" addlevel="always" swaplevel="1"/>
<gate name="-6" symbol="M" x="0" y="-5.08" addlevel="always" swaplevel="1"/>
<gate name="-7" symbol="M" x="0" y="-7.62" addlevel="always" swaplevel="1"/>
</gates>
<devices>
<device name="" package="22-23-2071">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8078638/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="22-23-2071" constant="no"/>
<attribute name="OC_FARNELL" value="1654534" constant="no"/>
<attribute name="OC_NEWARK" value="56H0445" constant="no"/>
<attribute name="POPULARITY" value="1" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="JST_1.25" library="con-molex" library_urn="urn:adsk.eagle:library:165" deviceset="53?-07" device="047" package3d_urn="urn:adsk.eagle:package:8078516/1"/>
<part name="JST_2.54" library="con-molex" library_urn="urn:adsk.eagle:library:165" deviceset="22-23-2071" device="" package3d_urn="urn:adsk.eagle:package:8078638/1"/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="JST_1.25" gate="-1" x="22.86" y="58.42" smashed="yes">
<attribute name="NAME" x="25.4" y="57.658" size="1.524" layer="91"/>
<attribute name="VALUE" x="22.098" y="59.817" size="1.778" layer="96"/>
</instance>
<instance part="JST_1.25" gate="-2" x="22.86" y="55.88" smashed="yes">
<attribute name="NAME" x="25.4" y="55.118" size="1.524" layer="95"/>
</instance>
<instance part="JST_1.25" gate="-3" x="22.86" y="53.34" smashed="yes">
<attribute name="NAME" x="25.4" y="52.578" size="1.524" layer="95"/>
</instance>
<instance part="JST_1.25" gate="-4" x="22.86" y="50.8" smashed="yes">
<attribute name="NAME" x="25.4" y="50.038" size="1.524" layer="95"/>
</instance>
<instance part="JST_1.25" gate="-5" x="22.86" y="48.26" smashed="yes">
<attribute name="NAME" x="25.4" y="47.498" size="1.524" layer="95"/>
</instance>
<instance part="JST_1.25" gate="-6" x="22.86" y="45.72" smashed="yes">
<attribute name="NAME" x="25.4" y="44.958" size="1.524" layer="95"/>
</instance>
<instance part="JST_1.25" gate="-7" x="22.86" y="43.18" smashed="yes">
<attribute name="NAME" x="25.4" y="42.418" size="1.524" layer="95"/>
</instance>
<instance part="JST_2.54" gate="-1" x="71.12" y="58.42" smashed="yes">
<attribute name="NAME" x="73.66" y="57.658" size="1.524" layer="95"/>
<attribute name="VALUE" x="70.358" y="59.817" size="1.778" layer="96"/>
</instance>
<instance part="JST_2.54" gate="-2" x="71.12" y="55.88" smashed="yes">
<attribute name="NAME" x="73.66" y="55.118" size="1.524" layer="95"/>
</instance>
<instance part="JST_2.54" gate="-3" x="71.12" y="53.34" smashed="yes">
<attribute name="NAME" x="73.66" y="52.578" size="1.524" layer="95"/>
</instance>
<instance part="JST_2.54" gate="-4" x="71.12" y="50.8" smashed="yes">
<attribute name="NAME" x="73.66" y="50.038" size="1.524" layer="95"/>
</instance>
<instance part="JST_2.54" gate="-5" x="71.12" y="48.26" smashed="yes">
<attribute name="NAME" x="73.66" y="47.498" size="1.524" layer="95"/>
</instance>
<instance part="JST_2.54" gate="-6" x="71.12" y="45.72" smashed="yes">
<attribute name="NAME" x="73.66" y="44.958" size="1.524" layer="95"/>
</instance>
<instance part="JST_2.54" gate="-7" x="71.12" y="43.18" smashed="yes">
<attribute name="NAME" x="73.66" y="42.418" size="1.524" layer="95"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="VCC" class="0">
<segment>
<pinref part="JST_1.25" gate="-2" pin="S"/>
<pinref part="JST_2.54" gate="-2" pin="S"/>
<wire x1="20.32" y1="55.88" x2="68.58" y2="55.88" width="0.1524" layer="91"/>
</segment>
</net>
<net name="OUT1" class="0">
<segment>
<pinref part="JST_1.25" gate="-3" pin="S"/>
<pinref part="JST_2.54" gate="-3" pin="S"/>
<wire x1="20.32" y1="53.34" x2="68.58" y2="53.34" width="0.1524" layer="91"/>
</segment>
</net>
<net name="OUT2" class="0">
<segment>
<pinref part="JST_1.25" gate="-4" pin="S"/>
<pinref part="JST_2.54" gate="-4" pin="S"/>
<wire x1="20.32" y1="50.8" x2="68.58" y2="50.8" width="0.1524" layer="91"/>
</segment>
</net>
<net name="SCK" class="0">
<segment>
<pinref part="JST_1.25" gate="-5" pin="S"/>
<pinref part="JST_2.54" gate="-5" pin="S"/>
<wire x1="20.32" y1="48.26" x2="68.58" y2="48.26" width="0.1524" layer="91"/>
</segment>
</net>
<net name="SI" class="0">
<segment>
<pinref part="JST_1.25" gate="-6" pin="S"/>
<pinref part="JST_2.54" gate="-6" pin="S"/>
<wire x1="20.32" y1="45.72" x2="68.58" y2="45.72" width="0.1524" layer="91"/>
</segment>
</net>
<net name="CS" class="0">
<segment>
<pinref part="JST_1.25" gate="-7" pin="S"/>
<pinref part="JST_2.54" gate="-7" pin="S"/>
<wire x1="20.32" y1="43.18" x2="68.58" y2="43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="JST_1.25" gate="-1" pin="S"/>
<pinref part="JST_2.54" gate="-1" pin="S"/>
<wire x1="20.32" y1="58.42" x2="68.58" y2="58.42" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
