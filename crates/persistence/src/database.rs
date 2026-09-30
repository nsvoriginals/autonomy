//! Database connection

use autonomy_common::error::Result;
use sqlx::{sqlite::SqlitePoolOptions, SqlitePool};
use std::path::Path;
use std::time::Duration;

/// Database connection pool
pub struct Database {
    pool: SqlitePool,
}

impl Database {
    pub async fn new(database_url: &str) -> Result<Self> {
        let pool = SqlitePoolOptions::new()
            .max_connections(10)
            .acquire_timeout(Duration::from_secs(30))
            .connect(database_url)
            .await?;
        
        Ok(Self { pool })
    }

    pub async fn new_in_memory() -> Result<Self> {
        Self::new("sqlite::memory:").await
    }

    pub async fn new_file(path: &Path) -> Result<Self> {
        let url = format!("sqlite://{}?mode=rwc", path.display());
        Self::new(&url).await
    }

    pub fn pool(&self) -> &SqlitePool {
        &self.pool
    }

    pub async fn migrate(&self) -> Result<()> {
        sqlx::migrate!("./migrations").run(&self.pool).await?;
        Ok(())
    }

    pub async fn close(&self) {
        self.pool.close().await;
    }
}

/// Transaction helper
pub struct Transaction<'a> {
    tx: sqlx::Transaction<'a, sqlx::Sqlite>,
}

impl<'a> Transaction<'a> {
    pub async fn begin(pool: &'a SqlitePool) -> Result<Self> {
        let tx = pool.begin().await?;
        Ok(Self { tx })
    }

    pub async fn commit(self) -> Result<()> {
        self.tx.commit().await?;
        Ok(())
    }

    pub async fn rollback(self) -> Result<()> {
        self.tx.rollback().await?;
        Ok(())
    }

    pub fn tx(&mut self) -> &mut sqlx::Transaction<'a, sqlx::Sqlite> {
        &mut self.tx
    }
}