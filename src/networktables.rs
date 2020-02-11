use nt::{Client, Entry, EntryData, EntryValue, NetworkTables};

pub const DEFAULT_ROBORIO_IP: &'static str = "10.49.4.2:1735";

pub async fn get_entry<'a>(
    nt: &'a NetworkTables<Client>,
    name: String,
    or_create_with: EntryValue,
) -> Entry<'a, Client> {
    nt.get_entry(
        match nt
            .entries()
            .iter()
            .find(|(_id, entry)| entry.name == name)
            .map(|(id, _entry)| *id)
        {
            Some(id) => id,
            None => {
                nt.create_entry(EntryData::new(name, 0, or_create_with))
                    .await
            }
        },
    )
}
