select json_extract(message,'$.electrical.solar.1.panelPower.value') as PanelPower from JrnlColibris where json_extract(message,'$.name') = 'Colibris';
select unix_timestamp(date),json_extract(message,'$.electrical.solar.1.panelPower.value') as PanelPower from JrnlColibris where json_extract(message,'$.name') = 'Colibris';

