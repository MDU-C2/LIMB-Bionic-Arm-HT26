Uppdatering av EMG-krets med inkluderad ESP samt IMU

Filer:
BOM_LIMB
En Bill of Material för komponenterna som skulle användas till den nya designen av EMG-PCBerna.
	-	Alla komponenter samt datablad är länkade.

EMG-Case-Bottom 
STL-fil för underdelen av det nya inkapslingen av den uppdaterade EMGn
	-	Tjockleken mellan bottens insida och urtagen för de elastiska banden är något tunn och kan göras 
		0.5-1mm tjockare för mer rubusthet.

EMG-Case-Lid
STL-fil for locket till den nya inkapslingen av den uppdaterade EMGn
	-	Notera att en mindre ändring av infällningen skulle förbättra lockets passform. 
		Genom att ta bort infällningen på långsidorna skulle hakarna falla in i underdelens urtag bättre.

EMG-FinalRoutingWithout3V
Den uppdaterade PCB-desigen i Ultiboard. 
	-	3.3V kretsen har tagits bort och kopplats direkt till 3.3V på ESPn.
		Detta då den ursprungliga modellen är konstruerad för 5V inspänning. 
	-	Kontakter har bytts ut till pins
	-	ESP och IMU har inkluderats i kretsen för enklare montering och stabilare signal
	-	PCBn gjordes mer avlång istället för kvadratisk då detta underlättar placering på arm
	
EMG-ForRouting
Denna multisim-fil är orörd och har den ursprungliga kretsen.
	-	Notera att all ändring av kretsen har gjorts direkt i Netlist i Ultiboard.
	-	Denna krets visar inte rätt värde på komponenterna.
	-	Används endast för att mappa namnet på komponenterna i EMG-FinalRoutingWithout3V till en 
		placering i kretsen.
		
Inspelning EMG-Case-Bottom
Video på CAD-assemblyt av inkapslingen. Dessa filer ligger zippade efter en "Pack-and-go" i mappen
"EMG-Case-Assembly"

Kopplingsschema
Bild på EMG-kretsen inklusive RLD-kretsen
	-	För att höja förstärkningen i kretsen skulle jag rekommentera att byta ut de två 500ohm
		resistorerna mellan pin 8 och 1 i AD623AR till två 100ohm. Detta skulle ge en förstärkning 
		på 501-1000. Skulle detta indusera brus i signalen skulle jag ta ner dessa till två 250ohm
		och öka förstärkningen från -1 till -2 i första delen av "Additional Amplification"
	
Kopplingsshema-PowerSection
Bild på EMG-kretsens PowerSection
	-	Hela 3.3V delen har tagit bort. 
	-	1.5V delen har istället 3.3V inspänning från ESPn
	
PCB-front och PCB-back
Bilder på hur de nya PCBerna skulle se ut med de nya gerberfilerna
	
Ultiboard Exports
Gerberfiler för tillverkning av nya EMG/IMU-PCBer

OBS! 
Simulering av kretsen försökte göras i tidigare kurs tillsammans med Martin, men då kretsen innehåller 
en RLD-del ger en simulering inga trovärdiga eller givande resultat. Därför har ingen ny simulering 
skapats.
